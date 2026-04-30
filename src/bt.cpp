//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include <cstring>

#include "bt.h"


#include "btstack_event.h"
#include "l2cap.h"
#include "gap.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "usb.h"
#include "utils.h"
#include "controller_profile.h"
#include "persist.h"
#include "bsp/board_api.h"
#include "pico/sync.h"
#include "classic/sdp_server.h"

#define MTU 672


static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_can_send_now_drain(void);

static btstack_packet_callback_registration_t hci_event_callback_registration, l2cap_event_callback_registration;
static bd_addr_t current_device_addr;
static bool device_found = false;
static bool new_pair = false; // 只有新匹配的设备才用创建channel，自动重连走的是service
static hci_con_handle_t acl_handle = HCI_CON_HANDLE_INVALID;
static uint16_t hid_control_cid;
static uint16_t hid_interrupt_cid;
static bt_data_callback_t bt_data_callback = nullptr;
// Cached feature-report payloads, keyed by report ID. Linear scan; cold path.
// Sized to cover DS (4-5 IDs) + Edge (~20 IDs) with headroom. Each entry stores
// just the payload — no report ID prefix.
#define FEATURE_CACHE_SLOTS 24
#define FEATURE_PAYLOAD_MAX 64
struct feature_entry {
    uint8_t  id;       // 0 = unused
    uint8_t  len;      // bytes valid in payload
    uint8_t  payload[FEATURE_PAYLOAD_MAX];
};
static feature_entry feature_table[FEATURE_CACHE_SLOTS];
static uint8_t feature_next_evict = 0;

static feature_entry* feature_find(uint8_t id) {
    for (uint8_t i = 0; i < FEATURE_CACHE_SLOTS; i++) {
        if (feature_table[i].id == id) return &feature_table[i];
    }
    return nullptr;
}

static feature_entry* feature_alloc(uint8_t id) {
    feature_entry* slot = feature_find(id);
    if (slot) return slot;
    // Find an unused slot first.
    for (uint8_t i = 0; i < FEATURE_CACHE_SLOTS; i++) {
        if (feature_table[i].id == 0) return &feature_table[i];
    }
    // Otherwise rotate-evict.
    slot = &feature_table[feature_next_evict];
    feature_next_evict = (feature_next_evict + 1) % FEATURE_CACHE_SLOTS;
    return slot;
}

static void feature_clear_all(void) {
    for (uint8_t i = 0; i < FEATURE_CACHE_SLOTS; i++) {
        feature_table[i].id = 0;
        feature_table[i].len = 0;
    }
    feature_next_evict = 0;
}

// Static slab pool for outbound L2CAP packets. Replaces std::queue<std::vector<uint8_t>>.
// 100 Hz audio path was malloc/free per frame; this keeps everything in BSS.
// Slot size 400 covers: 0x36 audio report (398 + A2 header = 399), 0x32 init (143),
// 0x31 forward output (79). Pool depth 8 absorbs short BT congestion bursts.
#define TX_SLOT_SIZE 400
#define TX_POOL_DEPTH 8
struct tx_slot {
    uint8_t  data[TX_SLOT_SIZE];
    uint16_t len;
};
// In scratch_x SRAM bank: hot path on every audio packet (~100 Hz),
// avoids striped-SRAM contention with core 1.
static tx_slot __scratch_x("bt_tx_pool") tx_pool[TX_POOL_DEPTH];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;
static uint8_t tx_count = 0;
volatile uint32_t bt_tx_drops = 0;
volatile uint32_t bt_tx_oversize = 0;
volatile uint32_t bt_l2cap_errs = 0;

static critical_section_t queue_lock;
uint32_t inactive_time = 0; // 手柄长时间静默

void bt_register_data_callback(bt_data_callback_t callback) {
    bt_data_callback = callback;
}

void bt_send_packet(uint8_t *data, uint16_t len) {
    if (hid_interrupt_cid != 0) {
        l2cap_send(hid_interrupt_cid, data, len);
    }
}

void bt_send_control(uint8_t *data, uint16_t len) {
    if (hid_control_cid != 0) {
        l2cap_send(hid_control_cid, data, len);
    }
}

bool bt_disconnect() {
    if (acl_handle == HCI_CON_HANDLE_INVALID) {
        return false;
    }

    // 0x13 = remote user terminated connection
    hci_send_cmd(&hci_disconnect, acl_handle, 0x13);
    return true;
}

void bt_l2cap_init() {
    l2cap_event_callback_registration.callback = &l2cap_packet_handler;
    l2cap_add_event_handler(&l2cap_event_callback_registration);
    // 修复重连后自动断开的关键点
    sdp_init();
    l2cap_register_service(l2cap_packet_handler, PSM_HID_CONTROL, MTU, LEVEL_2);
    l2cap_register_service(l2cap_packet_handler, PSM_HID_INTERRUPT, MTU, LEVEL_2);

    l2cap_init();
}

int bt_init() {
    critical_section_init(&queue_lock);

    bt_l2cap_init();

    // SSP (Secure Simple Pairing)
    gap_ssp_set_enable(true);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    gap_connectable_control(1);
    gap_discoverable_control(1);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    return 0;
}

/*int main() {
    stdio_init_all();

    /*while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("USB Serial connected!\n");#1#

    bt_init();

    while (1) {
        sleep_ms(10);
    }
}*/

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) channel;

    const uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
        case BTSTACK_EVENT_STATE: {
            const uint8_t state = btstack_event_state_get_state(packet);
            printf("[BT] State: %u\n", state);
            if (state == HCI_STATE_WORKING) {
                printf("[BT] Stack ready, start inquiry\n");
                gap_inquiry_start(30);
            }
            break;
        }
        case HCI_EVENT_INQUIRY_RESULT:
        case HCI_EVENT_INQUIRY_RESULT_WITH_RSSI:
        case HCI_EVENT_EXTENDED_INQUIRY_RESPONSE: {
            bd_addr_t addr;
            uint32_t cod;

            if (event_type == HCI_EVENT_INQUIRY_RESULT) {
                cod = hci_event_inquiry_result_get_class_of_device(packet);
                hci_event_inquiry_result_get_bd_addr(packet, addr);
            } else if (event_type == HCI_EVENT_INQUIRY_RESULT_WITH_RSSI) {
                cod = hci_event_inquiry_result_with_rssi_get_class_of_device(packet);
                hci_event_inquiry_result_with_rssi_get_bd_addr(packet, addr);
            } else {
                cod = hci_event_extended_inquiry_response_get_class_of_device(packet);
                hci_event_extended_inquiry_response_get_bd_addr(packet, addr);
            }

            // CoD 0x002508 = Gamepad (Major: Peripheral, Minor: Gamepad)
            if ((cod & 0x000F00) == 0x000500) {
                printf("[HCI] Gamepad found: %s (CoD: 0x%06x)\n", bd_addr_to_str(addr), (unsigned int) cod);
                bd_addr_copy(current_device_addr, addr);
                device_found = true;
                gap_inquiry_stop();
            }
            break;
        }

        case GAP_EVENT_INQUIRY_COMPLETE:
        case HCI_EVENT_INQUIRY_COMPLETE: {
            printf("[HCI] Inquiry complete\n");
            if (device_found) {
                printf("[HCI] Connecting to %s...\n", bd_addr_to_str(current_device_addr));
                new_pair = true;
                hci_send_cmd(&hci_create_connection, current_device_addr,
                             hci_usable_acl_packet_types(), 0, 0, 0, 1);
            }
            break;
        }
        case HCI_EVENT_COMMAND_STATUS: {
            const uint8_t status = hci_event_command_status_get_status(packet);
            const uint16_t opcode = hci_event_command_status_get_command_opcode(packet);
            printf("[HCI] CmdStatus %s(0x%04X) status=0x%02X\n", opcode_to_str(opcode), opcode, status);
            if (opcode == HCI_OPCODE_HCI_CREATE_CONNECTION && status != ERROR_CODE_SUCCESS) {
                device_found = false;
                new_pair = false;
                printf("[HCI] Create connection rejected, restart inquiry\n");
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_COMMAND_COMPLETE: {
            const uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
            const uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            printf("[HCI] CmdComplete %s(0x%04X) status=0x%02X\n", opcode_to_str(opcode), opcode, status);
            break;
        }

        case HCI_EVENT_CONNECTION_COMPLETE: {
            const uint8_t status = hci_event_connection_complete_get_status(packet);
            if (status == 0) {
                const hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
                acl_handle = handle;
                hci_event_connection_complete_get_bd_addr(packet, current_device_addr);
                printf("[HCI] ACL connected handle=0x%04X\n", handle);
                // Probe the remote name so we can pick the right controller profile
                // before USB enumerates. Reply arrives as HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE.
                gap_remote_name_request(current_device_addr, 0, 0);
                printf("[HCI] Request authentication on handle=0x%04X\n", handle);
                hci_send_cmd(&hci_authentication_requested, handle);
            } else {
                device_found = false;
                new_pair = false;
                printf("[HCI] ACL connect failed status=0x%02X, restart inquiry\n", status);
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
            const uint8_t status = hci_event_remote_name_request_complete_get_status(packet);
            if (status == 0) {
                const char *name = hci_event_remote_name_request_complete_get_remote_name(packet);
                const controller_profile_t *p = strstr(name, "Edge") ? &profile_dse : &profile_ds;
                controller_profile_set(p);
                printf("[BT] Remote name '%s' -> profile: %s\n", name, p->product_string);
                persist_save_if_changed(current_device_addr, p->profile_idx);
            } else {
                // Fall back to the default (DS) and warn. Edge users will see truncated
                // back-paddle bytes until they reconnect, but core functionality works.
                printf("[BT] Remote name request failed status=0x%02X, defaulting to DS\n", status);
            }
            break;
        }

        case HCI_EVENT_LINK_KEY_REQUEST: {
            bd_addr_t addr;
            hci_event_link_key_request_get_bd_addr(packet, addr);
            link_key_t link_key;
            link_key_type_t link_key_type;
            bool link = gap_get_link_key_for_bd_addr(addr, link_key, &link_key_type);
            printf("[HCI] Link key: ");
            for (int i = 0; i < sizeof(link_key_t); i++) {
                printf("%02X", link_key[i]);
            }
            printf("\n");
            if (link) {
                printf("[HCI] Link key request from %s, reply stored key type=%u\n", bd_addr_to_str(addr),
                       (unsigned int) link_key_type);
                hci_send_cmd(&hci_link_key_request_reply, addr, link_key);
            } else {
                printf("[HCI] Link key request from %s, no key, force re-pair\n", bd_addr_to_str(addr));
                hci_send_cmd(&hci_link_key_request_negative_reply, addr);
            }
            break;
        }

        case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
            bd_addr_t addr;
            hci_event_user_confirmation_request_get_bd_addr(packet, addr);
            printf("[HCI] User confirmation request from %s, accept\n", bd_addr_to_str(addr));
            hci_send_cmd(&hci_user_confirmation_request_reply, addr);
            break;
        }

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            printf("[HCI] Legacy pin request from %s, reply 0000\n", bd_addr_to_str(addr));
            gap_pin_code_response(addr, "0000");
            break;
        }

        case HCI_EVENT_AUTHENTICATION_COMPLETE: {
            const uint8_t status = hci_event_authentication_complete_get_status(packet);
            const hci_con_handle_t handle = hci_event_authentication_complete_get_connection_handle(packet);
            printf("[HCI] Authentication complete handle=0x%04X status=0x%02X\n", handle, status);
            if (status != ERROR_CODE_SUCCESS) {
                printf("[HCI] Authentication failed, drop stored key for %s\n", bd_addr_to_str(current_device_addr));
                gap_drop_link_key_for_bd_addr(current_device_addr);
                // gap_inquiry_start(30);
            } else {
                hci_send_cmd(&hci_set_connection_encryption, handle, 1);
            }
            break;
        }

        case HCI_EVENT_ENCRYPTION_CHANGE: {
            const uint8_t status = hci_event_encryption_change_get_status(packet);
            const hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
            const uint8_t enabled = hci_event_encryption_change_get_encryption_enabled(packet);
            printf("[HCI] Encryption change handle=0x%04X status=0x%02X enabled=%u\n", handle, status, enabled);
            if (status == ERROR_CODE_SUCCESS && enabled) {
                printf("[L2CAP] Open HID channels\n");
                if (new_pair) {
                    if (hid_control_cid == 0) {
                        l2cap_create_channel(l2cap_packet_handler, current_device_addr, PSM_HID_CONTROL, MTU,
                                             &hid_control_cid);
                    } else if (hid_interrupt_cid == 0) {
                        l2cap_create_channel(l2cap_packet_handler, current_device_addr, PSM_HID_INTERRUPT, MTU,
                                             &hid_interrupt_cid);
                    }
                }
            }
            break;
        }

        case HCI_EVENT_CONNECTION_REQUEST: {
            bd_addr_t addr;
            hci_event_connection_request_get_bd_addr(packet, addr);
            const uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
            printf("[HCI] Incoming ACL request from %s cod=0x%06x\n", bd_addr_to_str(addr), (unsigned int) cod);
            if ((cod & 0x000F00) == 0x000500) {
                bd_addr_copy(current_device_addr, addr);
                gap_inquiry_stop();
                hci_send_cmd(&hci_accept_connection_request, addr, 0x01);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            tud_disconnect();
            gap_connectable_control(1);
            gap_discoverable_control(1);
            const uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            device_found = false;
            new_pair = false;
            acl_handle = HCI_CON_HANDLE_INVALID;
            hid_control_cid = 0;
            hid_interrupt_cid = 0;
            feature_clear_all();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            printf("[HCI] Disconnected reason=0x%02X, start inquiry\n", reason);
            gap_inquiry_start(30);
            break;
        }
    }
}

static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) channel;

    if (packet_type == L2CAP_DATA_PACKET) {
        if (channel == hid_interrupt_cid) {
            // printf("[L2CAP] HID Interrupt data len=%u\n", size);
            // printf_hexdump(packet, size);
            bt_data_callback(INTERRUPT, packet, size);

            // 静默检测
            if (mute[1]) { // 麦克风静音开启
                return;
            }
            if (packet[3] < 120 || packet[3] > 140) {
                inactive_time = time_us_32();
            }else if (time_us_32() - inactive_time > 1800 * 1000 * 1000){
                printf("disconnect when inactive\n");
                inactive_time = time_us_32();
                bt_disconnect();
            }
        } else if (channel == hid_control_cid) {
            if (packet[0] == 0xA3 && size >= 2) {
                uint8_t report_id = packet[1];
                // Payload is everything after the 0xA3 header and the report id.
                uint16_t payload_len = (size > 2) ? (size - 2) : 0;
                if (payload_len > FEATURE_PAYLOAD_MAX) payload_len = FEATURE_PAYLOAD_MAX;
                feature_entry* slot = feature_alloc(report_id);
                slot->id = report_id;
                slot->len = (uint8_t)payload_len;
                if (payload_len > 0) memcpy(slot->payload, packet + 2, payload_len);
                printf("[L2CAP] Stored Feature Report 0x%02X, len=%u\n", report_id, payload_len);
            }
#ifdef DEBUG_BT
            printf("[L2CAP] HID Control data len=%u\n", size);
            printf_hexdump(packet, size);
#endif
            bt_data_callback(CONTROL, packet, size);
        } else {
            printf("[L2CAP] Data on unknown channel 0x%04X (Interrupt: 0x%04X, Control: 0x%04X)\n",
                   channel, hid_interrupt_cid, hid_control_cid);
        }
        return;
    }

    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case L2CAP_EVENT_CHANNEL_OPENED: {
            const uint8_t status = l2cap_event_channel_opened_get_status(packet);
            const uint16_t local_cid = l2cap_event_channel_opened_get_local_cid(packet);
            if (status == 0) {
                const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                if (psm == PSM_HID_CONTROL) {
                    printf("[L2CAP] HID Control opened cid=0x%04X\n", local_cid);
                    hid_control_cid = local_cid;
                } else if (psm == PSM_HID_INTERRUPT) {
                    printf("[L2CAP] HID Interrupt opened cid=0x%04X\n", local_cid);
                    hid_interrupt_cid = local_cid;

                    if (!mute[0]) {
                        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
                    }
                    inactive_time = time_us_32();

                    printf("Init DualSense\n");

                    init_feature();
                    // 初始化手柄状态
                    uint8_t report32[142];
                    report32[0] = 0x32;
                    report32[1] = 0x10; // reportSeqCounter
                    uint8_t packet_0x10[] =
                    {
                        0x90, // Packet: 0x10
                        0x3f, // 63
                        // SetStateData
                        0xfd, 0xf7, 0x0, 0x0,
                        0x7f, 0x7f, // Headphones, Speaker
                        0xff, 0x9, 0x0, 0xf, 0x0, 0x0, 0x0, 0x0,
                        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa,
                        0x7, 0x0, 0x0, 0x2, 0x1,
                        0x00,
                        0xff, 0xd7, 0x00 // RGB LED: R, G, B (Nijika Color!)✨
                    };
                    memcpy(report32 + 2, packet_0x10, sizeof(packet_0x10));
                    bt_write(report32, sizeof(report32));

                    tud_connect();
                } else {
                    printf("[L2CAP] Unknown Channel psm: 0x%02X", psm);
                }

                /*if (hid_control_cid != 0 && hid_interrupt_cid != 0) {
                    printf("[L2CAP] HID channels ready, request CAN_SEND_NOW for SET_PROTOCOL\n");
                    l2cap_request_can_send_now_event(hid_control_cid);
                }*/
            } else {
                const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                hid_control_cid = 0;
                hid_interrupt_cid = 0;
                device_found = false;
                printf("[L2CAP] Open failed psm=0x%04X status=0x%02X\n", psm, status);
                bt_disconnect();
            }
            break;
        }

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            const uint16_t local_cid = l2cap_event_incoming_connection_get_local_cid(packet);
            const uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            printf("[L2CAP] Incoming connection psm=0x%04X cid=0x%04X\n", psm, local_cid);
            l2cap_accept_connection(local_cid);
            break;
        }

        case L2CAP_EVENT_CHANNEL_CLOSED: {
            const uint16_t local_cid = l2cap_event_channel_closed_get_local_cid(packet);
            if (local_cid == hid_control_cid) {
                hid_control_cid = 0;
                printf("[L2CAP] HID Control closed cid=0x%04X\n", local_cid);
            } else if (local_cid == hid_interrupt_cid) {
                hid_interrupt_cid = 0;
                printf("[L2CAP] HID Interrupt closed cid=0x%04X\n", local_cid);
            } else {
                printf("[L2CAP] Channel closed cid=0x%04X\n", local_cid);
            }
            if (hid_control_cid == 0 && hid_interrupt_cid == 0) {
                bt_disconnect();
            }
            break;
        }

        case L2CAP_EVENT_CAN_SEND_NOW:
            l2cap_can_send_now_drain();
            break;
    }
}

static void __not_in_flash_func(l2cap_can_send_now_drain)(void) {
    critical_section_enter_blocking(&queue_lock);
    if (tx_count == 0) {
        critical_section_exit(&queue_lock);
        return;
    }
    // Snapshot the head slot under lock, send outside lock.
    uint8_t  local[TX_SLOT_SIZE];
    uint16_t local_len = tx_pool[tx_head].len;
    memcpy(local, tx_pool[tx_head].data, local_len);
    tx_head = (tx_head + 1) % TX_POOL_DEPTH;
    tx_count--;
    uint8_t remaining = tx_count;
    critical_section_exit(&queue_lock);

    uint8_t status = l2cap_send(hid_interrupt_cid, local, local_len);
    if (status != 0) {
        __atomic_fetch_add(&bt_l2cap_errs, 1, __ATOMIC_RELAXED);
    }
    if (remaining > 0) {
        // More packets queued -> re-arm to drain the next one.
        l2cap_request_can_send_now_event(hid_interrupt_cid);
    }
}

void __not_in_flash_func(bt_write)(uint8_t *data, uint16_t len) {
    if (hid_interrupt_cid == 0) return;
    if (len + 1 > TX_SLOT_SIZE) {
        __atomic_fetch_add(&bt_tx_oversize, 1, __ATOMIC_RELAXED);
        return;
    }

    critical_section_enter_blocking(&queue_lock);
    if (tx_count == TX_POOL_DEPTH) {
        // Drop oldest to keep the audio path moving under congestion.
        tx_head = (tx_head + 1) % TX_POOL_DEPTH;
        tx_count--;
        __atomic_fetch_add(&bt_tx_drops, 1, __ATOMIC_RELAXED);
    }
    tx_slot* slot = &tx_pool[tx_tail];
    slot->data[0] = 0xA2;
    memcpy(slot->data + 1, data, len);
    fill_output_report_checksum(slot->data + 1, len);
    slot->len = len + 1;
    tx_tail = (tx_tail + 1) % TX_POOL_DEPTH;
    tx_count++;
    bool was_only = (tx_count == 1);
    critical_section_exit(&queue_lock);

    if (was_only) {
        l2cap_request_can_send_now_event(hid_interrupt_cid);
    }
}

uint16_t get_feature_data(uint8_t reportId, uint16_t reqlen, uint8_t* out, uint16_t out_max) {
    // Look up cache; report 0x81 always re-queries (matches prior behavior).
    feature_entry* slot = feature_find(reportId);
    uint16_t copied = 0;
    if (slot && out && out_max > 0) {
        copied = slot->len < out_max ? slot->len : out_max;
        memcpy(out, slot->payload, copied);
    }
    if (!slot || reportId == 0x81) {
        if (hid_control_cid != 0) {
            uint8_t get_feature[] = {0x43, reportId};
            l2cap_send(hid_control_cid, get_feature, reqlen);
            printf("[L2CAP] Requesting Get Feature Report 0x%02X\n", reportId);
        }
    }
    return copied;
}

void set_feature_data(uint8_t reportId, uint8_t* data,uint16_t len) {
    if (hid_control_cid == 0) return;
    // Largest known DualSense / Edge feature report is 64 bytes; this stack
    // buffer covers any of them (header + report id + payload).
    static const uint16_t SET_FEATURE_MAX = 256;
    if (len + 2 > SET_FEATURE_MAX) {
        printf("[L2CAP] set_feature_data: len %u too large\n", len);
        return;
    }
    uint8_t buf[SET_FEATURE_MAX];
    buf[0] = 0x53;
    buf[1] = reportId;
    memcpy(buf + 2, data, len);
    fill_feature_report_checksum(buf + 1, len + 1);
    l2cap_send(hid_control_cid, buf, len + 2);
    printf("[L2CAP] Requesting Set Feature Report 0x%02X\n", reportId);
}

void init_feature() {
    // Fire-and-forget: cache will populate from the async control replies.
    get_feature_data(0x09, 20, nullptr, 0);
    get_feature_data(0x20, 64, nullptr, 0);
    get_feature_data(0x22, 64, nullptr, 0);
    get_feature_data(0x05, 41, nullptr, 0);
    if (g_profile == &profile_dse) {
        // Edge profile-select feature report. Pre-fetching cuts the first-poll
        // latency the host sees when probing Edge-only IDs.
        get_feature_data(0x60, 64, nullptr, 0);
    }
}

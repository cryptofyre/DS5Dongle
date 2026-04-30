//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "controller_profile.h"
#include "persist.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

int reportSeqCounter = 0;
uint8_t packetCounter = 0;

// Sized to fit Edge's longer input report; standard DualSense fits in the first 63 bytes.
// Initialized with a benign DualSense-shaped report so a host that polls before BT
// connects sees neutral inputs instead of zeroed-out (which presents as triggers held).
#define MAX_INPUT_REPORT 78
uint8_t interrupt_in_data[MAX_INPUT_REPORT] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

void interrupt_loop() {
    if (!tud_hid_ready()) return;

    bool should_send = false;
    uint8_t safe_report[MAX_INPUT_REPORT];
    uint16_t report_len = g_profile->input_report_size;

    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, report_len);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, report_len)) {
            printf("[USBHID] tud_hid_report error\n");

            // If the report failed to queue, restore the dirty flag
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void on_bt_data(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    if (channel == INTERRUPT && data[1] == 0x31) {
        // Headset bit lives at the same fixed offset in the BT-framed buffer for
        // both DS and Edge — header is 3 bytes, then bytes 53/56 of the report.
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }

        // Forward the controller-typed report length, but never more than what
        // the BT packet actually carries.
        uint16_t want = g_profile->input_report_size;
        uint16_t avail = (len > 3) ? (uint16_t)(len - 3) : 0;
        uint16_t copy = want < avail ? want : avail;
        if (copy > MAX_INPUT_REPORT) copy = MAX_INPUT_REPORT;

        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, copy);
        report_dirty = true;
        critical_section_exit(&report_cs);
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    (void) itf;
    (void) report_type;
    return get_feature_data(report_id, reqlen, buffer, reqlen);
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                uint8_t outputData[78];
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) {
                    reportSeqCounter = 0;
                }
                outputData[2] = 0x10;
                memcpy(outputData + 3, buffer + 1, bufsize - 1);
                bt_write(outputData, sizeof(outputData));
                break;
            }
        }
    }
    if (report_id == 0x80) {
        set_feature_data(report_id,const_cast<uint8_t *>(buffer),bufsize);
        return;
    }
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(320000, true);

    board_init();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    tud_disconnect();
    board_init_after_tusb();

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0;i < 6;i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            }else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
  
    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);

    // Load the persisted controller profile so USB descriptors enumerate as the
    // last-paired controller before the BT side reconnects.
    persist_load();

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();

    watchdog_enable(1000, true);

    absolute_time_t next_stats = make_timeout_time_ms(5000);
    while (1) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        audio_loop();
        interrupt_loop();

        if (absolute_time_diff_us(get_absolute_time(), next_stats) <= 0) {
            uint32_t a = audio_fifo_drops, o = opus_fifo_drops, m = opus_dequeue_misses;
            uint32_t tx_d = bt_tx_drops, tx_o = bt_tx_oversize, l2 = bt_l2cap_errs;
            uint32_t hw = audio_get_core1_stack_high_water_bytes();
            if (a | o | m | tx_d | tx_o | l2) {
                printf("[Stats] audio_drops=%lu opus_drops=%lu opus_miss=%lu "
                       "bt_tx_drops=%lu bt_tx_oversize=%lu l2cap_errs=%lu c1_stack=%lu\n",
                       (unsigned long)a, (unsigned long)o, (unsigned long)m,
                       (unsigned long)tx_d, (unsigned long)tx_o, (unsigned long)l2,
                       (unsigned long)hw);
                audio_fifo_drops = 0;
                opus_fifo_drops = 0;
                opus_dequeue_misses = 0;
                bt_tx_drops = 0;
                bt_tx_oversize = 0;
                bt_l2cap_errs = 0;
            }
            next_stats = make_timeout_time_ms(5000);
        }
    }
}

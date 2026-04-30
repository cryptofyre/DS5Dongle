// Runtime DualSense vs DualSense Edge profile.
// Set once at BT-connect time (before tud_connect()), read by TinyUSB callbacks
// and by the BT->USB forward path. No lock needed — single writer, frozen
// before USB enumeration.

#ifndef DS5_BRIDGE_CONTROLLER_PROFILE_H
#define DS5_BRIDGE_CONTROLLER_PROFILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t       pid;                 // 0x0CE6 / 0x0DF2
    const uint8_t* hid_descriptor;
    uint16_t       hid_descriptor_len;  // 289 / 405
    const char*    product_string;
    uint16_t       input_report_size;   // BT->USB payload bytes (excludes report ID byte)
    uint16_t       output_report_size;  // USB->BT payload size, including BT framing
    uint8_t        profile_idx;         // 0 = standard, 1 = Edge (used for persistence + serial)
} controller_profile_t;

extern const controller_profile_t profile_ds;
extern const controller_profile_t profile_dse;
extern const controller_profile_t* g_profile;  // BSS, default = &profile_ds

void controller_profile_set(const controller_profile_t* p);
const controller_profile_t* controller_profile_for_idx(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif // DS5_BRIDGE_CONTROLLER_PROFILE_H

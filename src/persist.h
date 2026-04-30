// Last-pair persistence: stores BD address + controller profile index in the
// final flash sector so the dongle can pre-warm the correct USB descriptor
// across power cycles.

#ifndef DS5_BRIDGE_PERSIST_H
#define DS5_BRIDGE_PERSIST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Read the persisted profile and apply it to g_profile. Safe to call before
// bt_init() and TinyUSB enumeration. Silently no-ops if no valid record.
void persist_load(void);

// Write the (addr, profile_idx) tuple to flash if it differs from what's
// already stored. Skips the write entirely if unchanged — keeps wear at
// effectively zero in steady-state.
void persist_save_if_changed(const uint8_t addr[6], uint8_t profile_idx);

#ifdef __cplusplus
}
#endif

#endif // DS5_BRIDGE_PERSIST_H

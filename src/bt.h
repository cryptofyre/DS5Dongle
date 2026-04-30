//
// Created by awalol on 2026/3/4.
//

#ifndef DS5_BRIDGE_BT_H
#define DS5_BRIDGE_BT_H

#include <cstdint>

enum CHANNEL_TYPE {
    INTERRUPT,
    CONTROL
};

typedef void (*bt_data_callback_t)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len);

int bt_init();
void bt_register_data_callback(bt_data_callback_t callback);
void bt_send_packet(uint8_t *data, uint16_t len);
void bt_send_control(uint8_t *data, uint16_t len);
void bt_write(uint8_t* data,uint16_t len);

// Look up a cached feature report payload (excludes the report-id byte).
// Returns the number of bytes copied into `out`. If we don't have a cached
// reply (or `reportId == 0x81` which always re-queries), kicks off a BT
// GET_FEATURE request as a side effect and returns 0.
uint16_t get_feature_data(uint8_t reportId, uint16_t reqlen, uint8_t* out, uint16_t out_max);

void init_feature();
void set_feature_data(uint8_t reportId, uint8_t* data,uint16_t len);

extern volatile uint32_t bt_tx_drops;
extern volatile uint32_t bt_tx_oversize;
extern volatile uint32_t bt_l2cap_errs;

#endif //DS5_BRIDGE_BT_H
//
// Created by awalol on 2026/3/5.
//

#ifndef DS5_BRIDGE_AUDIO_H
#define DS5_BRIDGE_AUDIO_H

#include <cstdint>

void audio_init();
void audio_loop();
void core1_entry();
void set_headset(bool state);

extern volatile uint32_t audio_fifo_drops;
extern volatile uint32_t opus_fifo_drops;
extern volatile uint32_t opus_dequeue_misses;

// Highest core 1 stack depth ever observed since boot, in bytes. Use to
// right-size CORE1_STACK_WORDS in audio.cpp once measured.
uint32_t audio_get_core1_stack_high_water_bytes();

#endif //DS5_BRIDGE_AUDIO_H
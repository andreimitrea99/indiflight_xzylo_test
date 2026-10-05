/*
 * On-board sine generator for bench-testing one motor while DISARMED.
 * Controlled with MSP command MSP_SINE_TEST (233):
 *   payload: uint8 motor index, uint16 mean, uint16 amplitude, uint16 frequency [mHz]
 *   mean and amplitude in MSP motor units (1000 = stop, 2000 = full).
 * Stops when amplitude is 0, or when the motor is stopped normally
 * (MSP_SET_MOTOR with 1000, or the Motors tab).
 */

#pragma once

#include <stdint.h>
#include "common/time.h"

#define MSP_SINE_TEST 233

void sineTestSet(uint8_t motorIndex, uint16_t mean, uint16_t amplitude, uint16_t freqMilliHz);
void sineTestApply(timeUs_t currentTimeUs);

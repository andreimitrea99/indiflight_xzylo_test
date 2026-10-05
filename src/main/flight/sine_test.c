/*
 * On-board sine generator for bench-testing one motor while DISARMED.
 * See sine_test.h for the MSP interface.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"
#include "common/time.h"
#include "drivers/motor.h"
#include "flight/mixer.h"

#include "flight/sine_test.h"

static bool enabled = false;
static uint8_t motorIdx = 0;
static float meanOut = 0.0f;      // in motor output units (DShot value)
static float ampOut = 0.0f;
static float freqHz = 0.0f;
static float phase = 0.0f;        // rad, kept in [0, 2 pi)
static timeUs_t lastUs = 0;
static float stopOut = 0.0f;

void sineTestSet(uint8_t motorIndex, uint16_t mean, uint16_t amplitude, uint16_t freqMilliHz)
{
    stopOut = motorConvertFromExternal(1000);

    const int lo = (int)mean - (int)amplitude;
    const int hi = (int)mean + (int)amplitude;
    if (motorIndex >= MAX_SUPPORTED_MOTORS || amplitude == 0 || lo <= 1000 || hi > 2000) {
        // invalid request or explicit off: stop the sine and the motor
        if (enabled) {
            motor_disarmed[motorIdx] = stopOut;
        }
        enabled = false;
        return;
    }

    const float outLo = motorConvertFromExternal((uint16_t)lo);
    const float outHi = motorConvertFromExternal((uint16_t)hi);

    motorIdx = motorIndex;
    meanOut = 0.5f * (outLo + outHi);
    ampOut = 0.5f * (outHi - outLo);
    freqHz = freqMilliHz / 1000.0f;
    phase = 0.0f;
    lastUs = 0;

    // Mark the motor as running: keeps the normal disarmed path spinning it at the mean,
    // and makes Blackbox (blackbox_mode = MOTOR_TEST) start logging.
    motor_disarmed[motorIdx] = meanOut;
    enabled = true;
}

// Called every loop while disarmed, after motor[] was filled from motor_disarmed[].
void sineTestApply(timeUs_t currentTimeUs)
{
    if (!enabled) {
        return;
    }
    if (motor_disarmed[motorIdx] == stopOut) {
        // motor was stopped through MSP_SET_MOTOR or the Motors tab: sine off
        enabled = false;
        return;
    }

    if (lastUs != 0) {
        const float dt = cmpTimeUs(currentTimeUs, lastUs) * 1e-6f;
        phase += 2.0f * M_PIf * freqHz * dt;
        while (phase >= 2.0f * M_PIf) {
            phase -= 2.0f * M_PIf;
        }
    }
    lastUs = currentTimeUs;

    motor[motorIdx] = meanOut + ampOut * sinf(phase);
}

#ifndef TEST1_MODE_CONFIG_H
#define TEST1_MODE_CONFIG_H

/* CPU0 CCU60_CH0. The existing encoder and PI both require a 10 ms tick. */
#define TEST_TICK_MS                10U
#define TEST_DISPLAY_MS             200U    /* Foreground IPS refresh. */

/* Wheel-off-ground open-loop defaults, in Seekfree PWM units (10000 = 100%). */
#define TEST_PWM_START              300
#define TEST_PWM_STEP               100
#define TEST_PWM_MAX                1200
#define TEST_PWM_INTERVAL_MS        500U    /* Time per duty level. */
#define TEST_OPEN_LOOP_TIMEOUT_MS   10000U  /* From mode admission, including coast. */

/* Select each logical wheel direction: +1 forward, -1 reverse, 0 disabled.
 * Each wheel can override the common ramp without changing the other wheel. */
#define TEST_LEFT_DIRECTION         1
#define TEST_LEFT_PWM_START         TEST_PWM_START
#define TEST_LEFT_PWM_STEP          TEST_PWM_STEP
#define TEST_LEFT_PWM_MAX           TEST_PWM_MAX
#define TEST_RIGHT_DIRECTION        1
#define TEST_RIGHT_PWM_START        TEST_PWM_START
#define TEST_RIGHT_PWM_STEP         TEST_PWM_STEP
#define TEST_RIGHT_PWM_MAX          TEST_PWM_MAX

#define TEST_TARGET_MPS             (0.20f) /* Shared left/right PI target, m/s. */
#define TEST_VISION_TARGET_MPS      (0.12f) /* Initial field trial; 0 = steering only. */
#define TEST_VISION_MAX_MPS         (0.20f) /* Chassis trial ceiling, m/s. */
#define TEST_VISION_ACQUIRE_MS      1200U   /* Fresh frames but no usable track, ms. */
#define TEST_SWITCH_PAUSE_MS        300U    /* Minimum zero-output dwell before any drive. */
#define TEST_STILL_MS               200U    /* Consecutive pulse-free observation window. */
#define TEST_COAST_TIMEOUT_MS       2000U   /* Failure to settle before drive -> fault. */
#endif

#ifndef TEST1_MODE_H
#define TEST1_MODE_H

#include "encoder.h"
#include "motor.h"

typedef enum
{
    TEST_IDLE = 0,
    TEST_MOTOR_OPEN_LOOP,
    TEST_SPEED_CLOSED_LOOP,
    TEST_VISION_TRACK,
    TEST_FAULT
} test_state_t;

typedef enum
{
    TEST_NO_FAULT = 0,
    TEST_ESTOP,
    TEST_COAST_FAULT,
    TEST_REPORTED_FAULT,
    TEST_CONFIG_FAULT,
    TEST_CAMERA_INIT_FAULT,
    TEST_FRAME_TIMEOUT,
    TEST_LINE_LOST,
    TEST_CROSS_TIMEOUT
} test_fault_t;

typedef struct
{
    test_state_t state;             /* State committed by the control tick. */
    test_fault_t fault;             /* Cause of inhibition. */
    uint8 waiting;                  /* Drive selected; both outputs held at zero. */
    uint8 vision_hold;              /* First invalid running frame, outputs inhibited. */
    int32 servo_duty;               /* Absolute duty last commanded by chassis control. */
    uint8 estop;                    /* Live independent KEY1 latch. */
    uint8 reset_ready;              /* Software checks for manual reset are satisfied. */
    float target_mps;               /* Common L/R setpoint, m/s; zero outside closed loop. */
    float ramped_mps;               /* Reference currently fed to both PI loops, m/s. */
    motor_state_t left_motor;       /* Requested/applied signed duty units. */
    motor_state_t right_motor;
    encoder_snapshot_t encoder;     /* One paired encoder sample. */
} test_status_t;

/* Single-use ADS watch command. Operator writes 1 only after verifying standstill.
 * Foreground consumes the command on its next poll, including rejected commands. */
extern volatile uint8 test_manual_reset_request;
extern volatile uint32 test_control_max_ticks; /* ADS Watch, divide by STM0 frequency for seconds. */

void test_mode_init(void);          /* Call once after drivers, before starting PIT. */
void test_mode_poll(void);          /* CPU0 foreground: emergency, keys and manual reset. */
void test_mode_tick_10ms(void);     /* CCU60_CH0 only, after the single encoder sample. */
void test_mode_request_fault(void); /* CPU0 fault report: inhibit drive by next tick. */
uint32 test_mode_time_ms(void);     /* Unsigned wrapping control clock, ms. */
test_status_t test_mode_status(void); /* Coherent copy for foreground display. */
#endif

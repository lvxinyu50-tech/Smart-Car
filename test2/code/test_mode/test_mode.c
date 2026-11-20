#include "test_mode.h"
#include "test_mode_config.h"
#include "smartcar_param.h"
#include "speed_control.h"
#include "vision_track.h"
#include "IfxStm.h"
#include "key.h"
#include "steer.h"
#include "led.h"
#include "zf_common_interrupt.h"

#if TEST_TICK_MS != 10 || ENCODER_SAMPLE_MS != TEST_TICK_MS || KEY_SCAN_PERIOD_MS != 10
#error test1 requires the existing 10 ms encoder, key and PI schedules
#endif
#if TEST_LEFT_DIRECTION < -1 || TEST_LEFT_DIRECTION > 1 || TEST_RIGHT_DIRECTION < -1 || TEST_RIGHT_DIRECTION > 1
#error Wheel directions must be -1, 0 or +1
#endif
#define VALID_RAMP(start, step, max_pwm) ((start) >= 0 && (step) > 0 && (step) <= PWM_MAX && (start) <= (max_pwm) && (max_pwm) <= PWM_MAX)
#if !VALID_RAMP(TEST_LEFT_PWM_START, TEST_LEFT_PWM_STEP, TEST_LEFT_PWM_MAX) || !VALID_RAMP(TEST_RIGHT_PWM_START, TEST_RIGHT_PWM_STEP, TEST_RIGHT_PWM_MAX)
#error Check each wheel ramp and the hardware PWM limit
#endif
#if TEST_PWM_INTERVAL_MS < TEST_TICK_MS || TEST_PWM_INTERVAL_MS % TEST_TICK_MS || TEST_SWITCH_PAUSE_MS < TEST_TICK_MS || TEST_STILL_MS < TEST_TICK_MS || TEST_COAST_TIMEOUT_MS <= TEST_SWITCH_PAUSE_MS || TEST_COAST_TIMEOUT_MS <= TEST_STILL_MS || TEST_OPEN_LOOP_TIMEOUT_MS <= TEST_SWITCH_PAUSE_MS || TEST_OPEN_LOOP_TIMEOUT_MS <= TEST_STILL_MS
#error Invalid step, dwell, standstill or timeout configuration
#endif

/* CPU0 foreground posts desired modes. Only the control tick commits transitions
 * and commands actuators after initialization. Save/restore the CPU0 interrupt
 * mask around admission, latch checks and PWM commits; never draw under this lock. */
static volatile test_state_t state, desired;
static volatile test_fault_t hard_fault;    /* Persists until MCU reset. */
static volatile uint8 waiting, delivered;   /* Zero-output entry phase / stop receipt. */
static volatile uint32 clock_ms, quiet_ms;  /* PIT clock / consecutive stationary time. */
static uint32 vision_generation, drive_bad_count, drive_cross_epoch;
static uint32 entry_capture, drive_loss_epoch; /* Vision entry fence / loss trip counter. */
static volatile uint8 vision_hold;
static volatile int32 servo_duty = STEER_MIDDLE;
static uint32 entered_ms, drive_started_ms; /* ISR-only timestamps, ms. */
volatile uint8 test_manual_reset_request;
volatile uint32 test_control_max_ticks; /* Max observed masked control work, STM0 ticks. */

static uint8 keys_released(void)
{
    return gpio_get_level(KEY_1_PIN) && gpio_get_level(KEY_2_PIN) &&
           gpio_get_level(KEY_3_PIN) && gpio_get_level(KEY_4_PIN);
}

/* Called only with the CPU0 lock held. Each transition resets the old controller,
 * commands zero on both wheels, centers steering and starts a new dwell window. */
static void enter_state(test_state_t next)
{
    speed_control_stop();  /* Resets both integrals and calls motor_duty(0, 0). */
    steer_center();
    servo_duty = STEER_MIDDLE;
    vision_hold = 0;
    led_set(LED4, GPIO_HIGH);
    if (state == TEST_VISION_TRACK || next == TEST_VISION_TRACK)
        vision_generation = vision_track_new_session();
    state = desired = next;
    waiting = (next == TEST_MOTOR_OPEN_LOOP || next == TEST_SPEED_CLOSED_LOOP || next == TEST_VISION_TRACK);
    if (next == TEST_VISION_TRACK)
        entry_capture = vision_track_capture_id();
    entered_ms = clock_ms;
    quiet_ms = 0;
}

void test_mode_init(void)
{
    speed_control_init();
    if (!(TEST_TARGET_MPS > 0.0f && TEST_TARGET_MPS <= SPEED_TARGET_MAX_MPS &&
          vision_config_valid() && VISION_SPEED_CAP_MPS <= SPEED_TARGET_MAX_MPS &&
          VISION_STRAIGHT_MPS > 0 && VISION_STRAIGHT_MPS <= VISION_SPEED_CAP_MPS &&
          VISION_CURVE_MPS > 0 && VISION_CURVE_MPS <= VISION_SPEED_CAP_MPS &&
          VISION_CROSS_MPS > 0 && VISION_CROSS_MPS <= VISION_SPEED_CAP_MPS &&
          VISION_STEER_KP >= 0 && VISION_STEER_KD >= 0))
        hard_fault = TEST_CONFIG_FAULT;
    enter_state(hard_fault ? TEST_FAULT : TEST_IDLE);
}

void test_mode_request_fault(void)
{
    uint32 mask = interrupt_global_disable();
    if (hard_fault == TEST_NO_FAULT)
        hard_fault = TEST_REPORTED_FAULT;
    desired = TEST_FAULT;
    interrupt_global_enable(mask);
}

/* Foreground and status use this under lock. The key driver makes the final
 * debounced pin and delivery checks atomically in key_reset_emergency_stop(). */
static uint8 can_reset(void)
{
    uint8 visual = hard_fault == TEST_CAMERA_INIT_FAULT || hard_fault == TEST_FRAME_TIMEOUT ||
                   hard_fault == TEST_LINE_LOST || hard_fault == TEST_CROSS_TIMEOUT;
    uint8 estop = key_emergency_stop_requested();
    return state == TEST_FAULT && (visual || (estop && hard_fault == TEST_NO_FAULT)) &&
           (!estop || delivered) && quiet_ms >= TEST_STILL_MS && keys_released();
}

void test_mode_poll(void)
{
    uint32 mask = interrupt_global_disable();
    uint8 estop = key_emergency_stop_requested(); /* Always check before ordinary keys. */
    key_event_t event = key_read_event(KEY_READER_CONTROL);
    uint8 manual = test_manual_reset_request;
    test_manual_reset_request = 0;  /* Never defer an early reset request. */

    if (estop)
    {
        desired = TEST_FAULT;
        if (event == KEY_EVENT_1)
        {
            delivered = 1;
            manual = 0; /* Delivery and deliberate rearm must be separate operations. */
        }
    }
    if (manual == 1 && can_reset() && (!estop || key_reset_emergency_stop()))
    {
        while (key_read_event(KEY_READER_CONTROL) != KEY_EVENT_NONE) { }
        hard_fault = TEST_NO_FAULT;
        delivered = 0;
        desired = TEST_IDLE;
    }
    else if (!estop && state != TEST_FAULT && hard_fault == TEST_NO_FAULT)
    {
        switch (event)
        {
        case KEY_EVENT_2: desired = TEST_MOTOR_OPEN_LOOP; break;
        case KEY_EVENT_3: desired = TEST_SPEED_CLOSED_LOOP; break;
        case KEY_EVENT_4: desired = (state == TEST_VISION_TRACK) ? TEST_IDLE : TEST_VISION_TRACK; break;
        default: break;
        }
    }
    interrupt_global_enable(mask);
}

/* Previous interval must have zero wheel commands and no encoder counts.
 * For an emergency reset the same window must also contain only released keys. */
static void observe_standstill(void)
{
    if (get_left_encoder() || get_right_encoder() ||
        motor_get_left_applied_pwm() || motor_get_right_applied_pwm() ||
        (state == TEST_FAULT && !keys_released()))
        quiet_ms = 0;
    else if (quiet_ms < TEST_STILL_MS)
        quiet_ms += TEST_TICK_MS;
}

static int wheel_pwm(uint32 step_index, int start, int step, int limit, int direction)
{
    /* Bound the step index before multiplying, even at a very long run time. */
    int magnitude = limit;
    if (step_index <= (uint32)((limit - start) / step))
        magnitude = start + (int)step_index * step;
    return direction * magnitude;
}

static void vision_fault(test_fault_t cause)
{
    hard_fault = cause;
    enter_state(TEST_FAULT);
}

/* Called only by the locked 10 ms control tick. A completed frame that began
 * before the KEY4 transition cannot authorize drive, even if just processed. */
static uint8 vision_permits_drive(vision_result_t r, uint32 age)
{
    uint32 since_entry = r.capture_id - entry_capture;
    uint8 new_frame = r.has_frame && r.generation == vision_generation && since_entry != 0 && since_entry < 0x80000000UL;
    if (!vision_track_camera_ok())
        vision_fault(TEST_CAMERA_INIT_FAULT);
    else if ((!new_frame && age >= VISION_FRAME_TIMEOUT_MS) ||
             (new_frame && clock_ms - r.completed_ms >= VISION_FRAME_TIMEOUT_MS))
        vision_fault(TEST_FRAME_TIMEOUT);
    else if (!waiting && (r.cross_fault_epoch != drive_cross_epoch ||
             (r.crossing && clock_ms-r.cross_entered_ms >= VISION_CROSS_LIMIT_MS)))
        vision_fault(TEST_CROSS_TIMEOUT);
    else if (waiting && (!new_frame || !r.valid || r.state != VISION_NORMAL ||
                         r.normal_streak < VISION_START_FRAMES))
    {
        if (age >= VISION_ACQUIRE_MS)
            vision_fault(TEST_LINE_LOST);
        return 0;
    }
    else if (!waiting && drive_loss_epoch != r.loss_epoch)
        vision_fault(TEST_LINE_LOST);
    else if (!waiting && (vision_hold || r.bad_count != drive_bad_count || !r.valid))
    {
        if (!vision_hold)
        {
            speed_control_stop();
            steer_center();
            servo_duty = STEER_MIDDLE;
            vision_hold = 1;
        }
        return 0;
    }
    else
        return 1;
    return 0;
}

static void drive_tick(void)
{
    uint32 age = clock_ms - entered_ms;
    test_state_t active = state;
    vision_result_t frame;
    if (active == TEST_VISION_TRACK)
    {
        frame = vision_track_result();
        if (!vision_permits_drive(frame, age))
        {
            if (waiting) observe_standstill();
            return;
        }
    }
    if (active == TEST_MOTOR_OPEN_LOOP && age >= TEST_OPEN_LOOP_TIMEOUT_MS)
    {
        enter_state(TEST_IDLE);
        return;
    }
    if (waiting)
    {
        observe_standstill();
        if (age >= TEST_COAST_TIMEOUT_MS)
        {
            hard_fault = TEST_COAST_FAULT;
            enter_state(TEST_FAULT);
            return;
        }
        if (age < TEST_SWITCH_PAUSE_MS || quiet_ms < TEST_STILL_MS)
            return;
        waiting = 0;
        drive_started_ms = clock_ms;
        if (active == TEST_VISION_TRACK)
        {
            drive_loss_epoch = frame.loss_epoch;
            drive_bad_count = frame.bad_count;
            drive_cross_epoch = frame.cross_fault_epoch;
        }
        if (active == TEST_SPEED_CLOSED_LOOP || (active == TEST_VISION_TRACK && !VISION_STEER_ONLY))
        {
            speed_control_set_target(active == TEST_VISION_TRACK ? frame.target_mps : TEST_TARGET_MPS);
            speed_control_start();
        }
    }
    if (active == TEST_VISION_TRACK)
    {
        steer_set_offset(frame.steer_offset);
        servo_duty = STEER_MIDDLE + frame.steer_offset;
        if (!VISION_STEER_ONLY)
        {
            speed_control_set_target(frame.target_mps); /* Setter preserves the ramp and integrals. */
            speed_control_update_10ms();
        }
        vision_hold = 0;
    }
    else if (active == TEST_SPEED_CLOSED_LOOP)
        speed_control_update_10ms();
    else
    {
        uint32 i = (clock_ms - drive_started_ms) / TEST_PWM_INTERVAL_MS;
        motor_set_pwm(wheel_pwm(i, TEST_LEFT_PWM_START, TEST_LEFT_PWM_STEP,
                                TEST_LEFT_PWM_MAX, TEST_LEFT_DIRECTION),
                      wheel_pwm(i, TEST_RIGHT_PWM_START, TEST_RIGHT_PWM_STEP,
                                TEST_RIGHT_PWM_MAX, TEST_RIGHT_DIRECTION));
    }
}

void test_mode_tick_10ms(void)
{
    uint32 mask = interrupt_global_disable();
    uint32 start_ticks = IfxStm_getLower(&MODULE_STM0);
    uint32 elapsed;
    clock_ms += TEST_TICK_MS;
    /* A blocked foreground or a previously posted start cannot bypass this check. */
    if (key_emergency_stop_requested() || hard_fault != TEST_NO_FAULT)
        desired = TEST_FAULT;

    if (state != desired)
        enter_state(desired);
    else if (state == TEST_MOTOR_OPEN_LOOP || state == TEST_SPEED_CLOSED_LOOP || state == TEST_VISION_TRACK)
        drive_tick();
    else if (state == TEST_FAULT)
    {
        observe_standstill();
        if ((clock_ms - entered_ms) % TEST_DISPLAY_MS == 0)
            led_toggle(LED4);     /* Blinking is visible with either LED polarity. */
    }
    elapsed = IfxStm_getLower(&MODULE_STM0) - start_ticks;
    if (elapsed > test_control_max_ticks) test_control_max_ticks = elapsed;
    interrupt_global_enable(mask);
}

uint32 test_mode_time_ms(void) { return clock_ms; }

test_status_t test_mode_status(void)
{
    test_status_t s;
    uint32 mask = interrupt_global_disable();
    s.state = state;
    s.estop = key_emergency_stop_requested();
    s.fault = hard_fault ? hard_fault : (s.estop ? TEST_ESTOP : TEST_NO_FAULT);
    s.waiting = waiting;
    s.reset_ready = can_reset();
    s.vision_hold = vision_hold;
    s.servo_duty = servo_duty;
    s.target_mps = (state == TEST_VISION_TRACK && !waiting && !vision_hold && !VISION_STEER_ONLY) ? speed_control_get_target() :
                   ((state == TEST_SPEED_CLOSED_LOOP) ? TEST_TARGET_MPS : 0.0f);
    s.ramped_mps = speed_control_get_ramped_target();
    s.left_motor = motor_get_left_state();
    s.right_motor = motor_get_right_state();
    s.encoder = encoder_get_snapshot();
    interrupt_global_enable(mask);
    return s;
}

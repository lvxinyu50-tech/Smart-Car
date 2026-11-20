/*
 * Forward-only dual-wheel PI speed control for the first field test.
 * Called from the 10 ms PIT through car_control_update_10ms().
 * Initial gains are intentionally conservative and require ground tuning.
 */

#include "speed_control.h"
#include "smartcar_param.h"
#include "motor.h"
#include "encoder.h"

typedef struct
{
    float kp;
    float ki;
    float integral;
} speed_pi_t;

static speed_pi_t left_pi;
static speed_pi_t right_pi;

static volatile float target_speed_mps = CAR_TARGET_SPEED_MPS;
static volatile float ramped_speed_mps = 0;
static volatile int left_output = 0;
static volatile int right_output = 0;

static float speed_clamp(float value, float min_value, float max_value)
{
    if(value > max_value)
        value = max_value;
    else if(value < min_value)
        value = min_value;
    return value;
}

static void speed_pi_reset(speed_pi_t *controller)
{
    controller->integral = 0;
}

static int speed_pi_update(speed_pi_t *controller, float target, float measured)
{
    float error = target - measured;
    float candidate_integral = controller->integral + error * SPEED_CONTROL_DT_S;
    float candidate_output;
    float output;

    candidate_integral = speed_clamp(
        candidate_integral, -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);

    candidate_output = controller->kp * error + controller->ki * candidate_integral;

    // Conditional integration prevents windup at either output limit.
    if(((candidate_output > 0) && (candidate_output < SPEED_OUTPUT_LIMIT)) ||
       ((candidate_output >= SPEED_OUTPUT_LIMIT) && (error < 0)) ||
       ((candidate_output <= 0) && (error > 0)))
    {
        controller->integral = candidate_integral;
    }

    output = controller->kp * error + controller->ki * controller->integral;
    output = speed_clamp(output, 0, SPEED_OUTPUT_LIMIT);
    return (int)output;
}

void speed_control_init(void)
{
    left_pi.kp = SPEED_LEFT_KP;
    left_pi.ki = SPEED_LEFT_KI;
    right_pi.kp = SPEED_RIGHT_KP;
    right_pi.ki = SPEED_RIGHT_KI;

    target_speed_mps = CAR_TARGET_SPEED_MPS;
    speed_control_stop();
}

void speed_control_start(void)
{
    speed_pi_reset(&left_pi);
    speed_pi_reset(&right_pi);
    ramped_speed_mps = 0;
    left_output = 0;
    right_output = 0;
    motor_duty(0, 0);
}

void speed_control_stop(void)
{
    speed_pi_reset(&left_pi);
    speed_pi_reset(&right_pi);
    ramped_speed_mps = 0;
    left_output = 0;
    right_output = 0;
    motor_duty(0, 0);
}

void speed_control_update_10ms(void)
{
    const float ramp_step = SPEED_RAMP_MPS_PER_S * SPEED_CONTROL_DT_S;

    if(ramped_speed_mps < target_speed_mps)
    {
        ramped_speed_mps += ramp_step;
        if(ramped_speed_mps > target_speed_mps)
            ramped_speed_mps = target_speed_mps;
    }
    else if(ramped_speed_mps > target_speed_mps)
    {
        ramped_speed_mps -= ramp_step;
        if(ramped_speed_mps < target_speed_mps)
            ramped_speed_mps = target_speed_mps;
    }

    left_output = speed_pi_update(&left_pi, ramped_speed_mps, get_left_speed());
    right_output = speed_pi_update(&right_pi, ramped_speed_mps, get_right_speed());

    motor_duty(left_output, right_output);
}

void speed_control_set_target(float target_mps)
{
    target_speed_mps = speed_clamp(target_mps, 0, SPEED_TARGET_MAX_MPS);
}

float speed_control_get_target(void)
{
    return target_speed_mps;
}

float speed_control_get_ramped_target(void)
{
    return ramped_speed_mps;
}

int speed_control_get_left_output(void)
{
    return left_output;
}

int speed_control_get_right_output(void)
{
    return right_output;
}

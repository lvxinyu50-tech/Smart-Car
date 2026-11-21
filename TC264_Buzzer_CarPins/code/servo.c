#include "zf_common_headfile.h"

/* 50 Hz calibration; duty values use PWM_DUTY_MAX = 10000. */
float Steer_PWM_mid  = SERVO_PWMMID;
float Steer_PWM_Lmax = SERVO_PWMMAX;
float Steer_PWM_Rmax = SERVO_PWMMIN;
float Err_steer = 0.0f;
float steer_out = SERVO_PWMMID;

/* The original gains used duty counts at 300 Hz. At 50 Hz each duty
 * count changes pulse width six times as much, so scale every active gain
 * by the frequency ratio. Keep the calibrated center and limits unchanged.
 * These gains still use the existing 5 ms control interval.
 */
#define SERVO_GAIN_SCALE (SERVO_PWM_FREQ / 300.0f)

ServoPID ERROR_steer = {
    16.29f * SERVO_GAIN_SCALE,
    0.0f,
    4.0f  * SERVO_GAIN_SCALE,
    1.0f  * SERVO_GAIN_SCALE,
    0.09f * SERVO_GAIN_SCALE
};
static int16 previous_error = 0;

void servo_init(void)
{
    previous_error = 0;
    Err_steer = 0.0f;
    steer_out = Steer_PWM_mid;
    pwm_init(Steer_GPIO, SERVO_PWM_FREQ, (uint32)steer_out);
}

void steer_control(void)
{
    int image_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
    int16 error;

    /* First-car lane error replaces the Second-car bright-marker X_Error. */
    if (image_error > 65)
    {
        image_error = 65;
    }
    else if (image_error < -65)
    {
        image_error = -65;
    }
    error = (int16)image_error;

    Err_steer = ERROR_steer.Kp * error
              + ERROR_steer.Kd * (error - previous_error)
              + ERROR_steer.Kp_2 * error * abs(error)
              + ERROR_steer.Groy_kd * imu_z;
    previous_error = error;

    /* The target project uses Stop == 0 instead of start_flag == 1. */
    if (SystemData.Stop == 0)
    {
        steer_out = Steer_PWM_mid - Err_steer;
    }
    else
    {
        steer_out = Steer_PWM_mid;
    }

    if (steer_out < Steer_PWM_Rmax)
    {
        steer_out = Steer_PWM_Rmax;
    }
    if (steer_out > Steer_PWM_Lmax)
    {
        steer_out = Steer_PWM_Lmax;
    }
    pwm_set_duty(Steer_GPIO, (uint32)steer_out);
}

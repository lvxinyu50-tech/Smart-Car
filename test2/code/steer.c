#include "steer.h"
#include "zf_driver_pwm.h"

#if (STEER_R_MIN < 0) || (STEER_R_MIN > STEER_MIDDLE) || (STEER_MIDDLE > STEER_L_MAX) || (STEER_L_MAX > PWM_DUTY_MAX)
#error Steering center and limits must be ordered within the PWM range
#endif
#if (STEER_PWM_FREQ_HZ <= 0) || (STEER_PWM_FREQ_HZ > 300)
#error Steering PWM frequency is outside the configured servo range
#endif

void steer_init(void)
{
    pwm_init(STEER_PWM_CHANNEL, STEER_PWM_FREQ_HZ, STEER_MIDDLE);
}

void steer_set_duty(uint32 duty)
{
    if (duty < STEER_R_MIN)
        duty = STEER_R_MIN;
    else if (duty > STEER_L_MAX)
        duty = STEER_L_MAX;
    pwm_set_duty(STEER_PWM_CHANNEL, duty);
}

void steer_set_offset(int32 offset)
{
    /* Widen before adding so even INT32_MAX is a valid input. */
    int64 duty = (int64)STEER_MIDDLE + offset;
    if (duty < STEER_R_MIN)
        duty = STEER_R_MIN;
    else if (duty > STEER_L_MAX)
        duty = STEER_L_MAX;
    pwm_set_duty(STEER_PWM_CHANNEL, (uint32)duty);
}

void steer_center(void)            { steer_set_duty(STEER_MIDDLE); }
uint32 steer_get_center_duty(void) { return STEER_MIDDLE; }
void steer_duty(uint32 duty)       { steer_set_duty(duty); }

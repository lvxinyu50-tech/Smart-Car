#include "motor.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

#if (PWM_MAX <= 0) || (PWM_MAX > PWM_DUTY_MAX)
#error PWM_MAX must fit the Seekfree PWM range
#endif

static volatile motor_state_t left_state;    /* CPU0 foreground/ISR left command. */
static volatile motor_state_t right_state;   /* CPU0 foreground/ISR right command. */

/* All callers hold the CPU0 interrupt lock across PH, EN and state writes. */
static void apply_pwm(gpio_pin_enum ph, pwm_channel_enum en, int reverse,
                      int request, volatile motor_state_t *state)
{
    int applied = request;   /* Signed, in PWM duty units. */
    uint32 magnitude;        /* Nonnegative duty accepted by pwm_set_duty. */

    if (applied > PWM_MAX)
        applied = PWM_MAX;
    else if (applied < -PWM_MAX)
        applied = -PWM_MAX;
    if (reverse)
        applied = -applied;

    magnitude = (uint32)((applied < 0) ? -applied : applied);
    gpio_set_level(ph, (applied < 0) ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(en, magnitude);
    state->requested_pwm = request;
    state->applied_pwm = applied;
}

void motor_init(void)
{
    gpio_init(MOTOR_L_PH_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_PH_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_L_EN_PWM, MOTOR_PWM_FREQ_HZ, 0);
    pwm_init(MOTOR_R_EN_PWM, MOTOR_PWM_FREQ_HZ, 0);
    left_state.requested_pwm = left_state.applied_pwm = 0;
    right_state.requested_pwm = right_state.applied_pwm = 0;
}

void motor_set_left_pwm(int pwm)
{
    uint32 mask = interrupt_global_disable(); /* Previous CPU0 interrupt state. */
    apply_pwm(MOTOR_L_PH_PIN, MOTOR_L_EN_PWM, MOTOR_L_REVERSE, pwm, &left_state);
    interrupt_global_enable(mask);
}

void motor_set_right_pwm(int pwm)
{
    uint32 mask = interrupt_global_disable();
    apply_pwm(MOTOR_R_PH_PIN, MOTOR_R_EN_PWM, MOTOR_R_REVERSE, pwm, &right_state);
    interrupt_global_enable(mask);
}

void motor_set_pwm(int left, int right)
{
    uint32 mask = interrupt_global_disable();
    apply_pwm(MOTOR_L_PH_PIN, MOTOR_L_EN_PWM, MOTOR_L_REVERSE, left, &left_state);
    apply_pwm(MOTOR_R_PH_PIN, MOTOR_R_EN_PWM, MOTOR_R_REVERSE, right, &right_state);
    interrupt_global_enable(mask);
}

motor_state_t motor_get_left_state(void)
{
    uint32 mask = interrupt_global_disable();
    motor_state_t state = left_state;
    interrupt_global_enable(mask);
    return state;
}

motor_state_t motor_get_right_state(void)
{
    uint32 mask = interrupt_global_disable();
    motor_state_t state = right_state;
    interrupt_global_enable(mask);
    return state;
}

int motor_get_left_requested_pwm(void)  { return motor_get_left_state().requested_pwm; }
int motor_get_right_requested_pwm(void) { return motor_get_right_state().requested_pwm; }
int motor_get_left_applied_pwm(void)    { return motor_get_left_state().applied_pwm; }
int motor_get_right_applied_pwm(void)   { return motor_get_right_state().applied_pwm; }

void left_motor_duty(int duty)  { motor_set_left_pwm(duty); }
void right_motor_duty(int duty) { motor_set_right_pwm(duty); }
void motor_duty(int l, int r)   { motor_set_pwm(l, r); }
int get_left_duty(void)         { return motor_get_left_applied_pwm(); }
int get_right_duty(void)        { return motor_get_right_applied_pwm(); }

#ifndef SMARTCAR_MOTOR_H
#define SMARTCAR_MOTOR_H

#include "driver_config.h"

/* DRV8701E: PH selects direction, EN receives the unsigned PWM magnitude.
 * All values below use signed duty units (10000 units = 100% in this SDK). */
typedef struct
{
    int requested_pwm;  /* Original caller input, before limiting and reversal. */
    int applied_pwm;    /* Limited, direction-corrected command. PH high if negative. */
} motor_state_t;

void motor_init(void);                      /* Configure both channels at zero duty. */
void motor_set_left_pwm(int pwm);            /* Drive the left wheel only. */
void motor_set_right_pwm(int pwm);           /* Drive the right wheel only. */
void motor_set_pwm(int left, int right);     /* Commit a pair of commands on CPU0. */
motor_state_t motor_get_left_state(void);    /* Coherent copy of left request/output. */
motor_state_t motor_get_right_state(void);   /* Coherent copy of right request/output. */
int motor_get_left_requested_pwm(void);      /* Left caller input, before limiting/reversal. */
int motor_get_right_requested_pwm(void);     /* Right caller input, before limiting/reversal. */
int motor_get_left_applied_pwm(void);        /* Signed left command after limiting/reversal. */
int motor_get_right_applied_pwm(void);       /* Signed right command after limiting/reversal. */

/* Original entry points and signed, direction-corrected getter semantics. */
void left_motor_duty(int duty);              /* Set left request. */
void right_motor_duty(int duty);             /* Set right request. */
void motor_duty(int l, int r);               /* Set both requests. */
int get_left_duty(void);                     /* Last applied left PWM. */
int get_right_duty(void);                    /* Last applied right PWM. */

#endif

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_

#include "zf_driver_pwm.h"

/* Ported from Nailong_NO.1_Second_1/code/motor.c and motor.h. */
#define Steer_GPIO       (ATOM1_CH1_P33_9)
#define SERVO_PWM_FREQ   (50U)

/* Car calibration in 100 Hz duty units. Round to the nearest output count. */
#define SERVO_DUTY_FROM_100HZ(duty) (((duty) * SERVO_PWM_FREQ + 50U) / 100U)
#define SERVO_PWMMAX SERVO_DUTY_FROM_100HZ(1680U)  /* Left limit */
#define SERVO_PWMMID SERVO_DUTY_FROM_100HZ(1505U)  /* Straight ahead */
#define SERVO_PWMMIN SERVO_DUTY_FROM_100HZ(1330U)  /* Right limit */

/* Kept separate from the wheel PID type in motor.h. Ki is reserved. */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kp_2;
    float Groy_kd;
} ServoPID;

extern ServoPID ERROR_steer;
extern float Steer_PWM_mid;
extern float Steer_PWM_Lmax;
extern float Steer_PWM_Rmax;
extern float Err_steer;
extern float steer_out;

/* Initialize P33_9 at center before enabling the 5 ms interrupt. */
void servo_init(void);

/* Uses ImageStatus.Det_True - MiddleLine, imu_z and SystemData.Stop. */
void steer_control(void);

#endif

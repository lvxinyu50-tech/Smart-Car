#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

/* Wheel speed targets as a percentage of the stock profile (100 = stock). */
#define TRACK_SPEED_PERCENT 25

typedef struct PID
{
    float   Kp;            // 比例常数
    float   Ki;            // 积分常数
    float   Kd;            // 微分常数
    float   kp_m;          // 比例常数的最大值
    float   kd_m;          // 微分常数的最大值
    float   Current_Error; // 当前误差
    float   Last_Error;    // Error[-1]
    float   Pre_Error;     // Error[-2]
} PID;
extern int16 speed_L, speed_R,imu_z;
extern int16 Speed_Goal,Err;
extern int16 Speed_OUT_R, Speed_OUT_L;


//动态PD参数
extern float P_A;
extern float P_B;
extern float D_coe;
//选择pid模式
extern uint8_t pid_mode;
//二次P
extern float kp_two;
extern float kp_two2;
extern float GKd;
extern float GKd2;
void motor_enable_on_first_start(void);
void motor_control(void);
void get_Err(void);
void get_imu_Err(void);



extern  PID ERROR;
extern  PID ERROR_LIG;
extern  PID SPEED_L;
extern  PID SPEED_R;



extern int fuya_pwm;

#endif

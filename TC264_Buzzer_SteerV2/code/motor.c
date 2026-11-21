#include "motor.h"
#include "stdlib.h"

/* Overall wheel differential correction scale. */
#define MOTOR_DIFF_GAIN (0.15f)

int16 speed_L, speed_R, imu_z = 0,Err = 0,imu_Err=0;
int16 Speed_Goal = 0, Speed_OUT_R, Speed_OUT_L;

int protect=0;
//struct PID SPEED_L = {150,0.4,0};        //
//struct PID SPEED_R = {150,0.4,0};        //lk速度环

struct PID SPEED_L = {50,6,0};        //
struct PID SPEED_R = {50,6,0};        //lk速度环
struct PID ERROR = {0,0,0};//圆环位置式PID的Kp,Ki,Kd,Kp_m,Kd_m;   b,Kd_m用于模糊控制，没有用到

uint8_t pid_mode=2;//1->固定参数 2->动态参数 3->模糊参数

struct PID imu_pid = {0,0,0};//角速度环PD控制

/*********/

////角速度调参
//float GKd =0.020;//圆9
////float GKd2=0.0000;//普通
////
//////二次P参数
////float kp_two=0.15;// 0.4
////float kp_two2=0.000;
////float xishu=0.4;
////
////
////float P_A=0.02 ;
////float P_B=3.8;
//////动态D调参
//float D_coe=0.4;
//
///*********/
//
//
////二次P参数
//float kp_two=0.8;// 0.4
//float xishu=0.2;
//
//
//float P_A=0.09  ;
//float P_B=3.5;
//
////动态D调参










/*********/
//
////角速度调参
float GKd =0.023;//圆9
//float GKd2=0.0000;//普通
//
////二次P参数
//float kp_two=0.15;// 0.4
//float kp_two2=0.000;
//float xishu=0.4;
//
//
//float P_A=0.02 ;
//float P_B=3.8;
////动态D调参
float D_coe=0.3;


/*********/


//二次P参数
float kp_two=0.28;// 0.4
float xishu=0.2;


float P_A=0.013;//0.013
float P_B=3.5;//3.5

//动态D调参













//求绝对值
float abs_value(float a)
{
    if(a>=0) return   a ;
    else     return (-a);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void get_Err(void)
{
    static int16 Err_ek1 = 0, Err_ek2 = 0;
    //保存上次误差
    Err_ek2 = Err_ek1;
    //更新本次误差
    Err_ek1 = ImageStatus.Det_True - ImageStatus.MiddleLine;
    if(pid_mode==1)
    {
        //固定参数不用写代码
    }
     if(pid_mode==2)//当偏差较小时，Kp 较小 → 控制平稳
        // 当偏差较大时，Kp 迅速上升 → 提高纠偏能力，防止冲出赛道
    {
        //动态P公式
        ERROR.Kp=(Err_ek1*Err_ek1)*P_A+P_B;//图像偏差自动调整控制器增益
//        Speed_Goal=-(Err_ek1*Err_ek1)*P_A+150;

        //动态P限幅
        //if(ERROR.Kp>11.0)     ERROR.Kp=11.0;
        //else if(ERROR.Kp<-11) ERROR.Kp=-11.0;
        if(ERROR.Kp>5.0)     ERROR.Kp=5.0;
        else if(ERROR.Kp<-5) ERROR.Kp=-5.0;
        //动态D公式
        ERROR.Kd    =D_coe*ERROR.Kp;//保证 Kd 跟随 Kp 变化，维持系统的稳定性
        //避免单独调节两个参数带来的复杂性
        //在弯道或高速状态下增强抗抖动能力
//       if(abs(Err_ek1)>12)
//            {
//
//                ERROR.Kd=0.4;
//            }

        if(ImageStatus.Road_type == LeftCirque)
        {

               kp_two = 1.5;
               GKd = 0.025;
        }

           else if(ImageStatus.Road_type == RightCirque)
           {

               kp_two = 1.5;
               GKd = 0.025;

           }
           else
           {
               kp_two = 0.3;//0.25
               GKd = 0.028;
           }

    }


    Err = MOTOR_DIFF_GAIN * (ERROR.Kp * Err_ek1                //P
         + ERROR.Kd * (Err_ek1 - Err_ek2)     //D
         + kp_two   *  Err_ek1 * abs(Err_ek1) //二次P
         + GKd      *  imu_z//角速度
         );




}

void get_imu_Err(void)
{
    static int16 imu_Err_ek1 = 0, imu_Err_ek2 = 0;
    //保存上次误差
    imu_Err_ek2 = imu_Err_ek1;
    //更新本次误差
    imu_Err_ek1 = 40 - imu_z;
    //计算输出值
    imu_Err =  imu_pid.Kp *  imu_Err_ek1                //P
             + imu_pid.Kd * (imu_Err_ek1 - imu_Err_ek2);     //D
}



float PID_motor_L(PID *pp, float Now_Point,float SetPoint)
{
    static float Increase_L;
    pp->Current_Error = SetPoint-Now_Point;
    Increase_L += pp->Kp * (pp->Current_Error - pp->Last_Error)                      //比例项
              + pp->Ki * pp->Current_Error                                         //积分项
              + pp->Kd * (pp->Current_Error - 2 * pp->Last_Error + pp->Pre_Error); //微分项
    pp->Pre_Error  = pp->Last_Error;
    pp->Last_Error = pp->Current_Error;
    if(Increase_L<-9000)Increase_L =-9000;
    if(Increase_L> 9000)Increase_L = 9000;
    return Increase_L;
}

float PID_motor_R(PID *pp, float Now_Point,float SetPoint)
{
    static float Increase_R;
    pp->Current_Error = SetPoint-Now_Point;
    Increase_R += pp->Kp * (pp->Current_Error - pp->Last_Error)                      //比例项
              + pp->Ki * pp->Current_Error                                         //积分项
              + pp->Kd * (pp->Current_Error - 2 * pp->Last_Error + pp->Pre_Error); //微分项
    pp->Pre_Error  = pp->Last_Error;
    pp->Last_Error = pp->Current_Error;
    if(Increase_R<-9000)Increase_R =-9000;
    if(Increase_R> 9000)Increase_R = 9000;
    return Increase_R;
}

void Motor_Protect(void)//这个函数还没测试好
{
    static int protect=0;
    if(speed_L>10&&speed_R>10)protect=1;//防止启动的时候检测到停止
    if(((speed_L<10&&Speed_OUT_L>5000)||(speed_R<10 && Speed_OUT_R>5000) )&& protect==1)
    {
        Speed_OUT_L = 0;
        Speed_OUT_R = 0;
    }
}


int fuya_pwm=600;

/* EN stays a low GPIO until the foreground accepts the first KEY4 start. */
static volatile uint8 motor_pwm_ready;

void motor_enable_on_first_start(void)
{
    if (!motor_pwm_ready)
    {
        pwm_init(MOTOR_L, 5000, 0);
        pwm_init(MOTOR_R, 5000, 0);
        motor_pwm_ready = 1;
    }
}

void motor_control(void)
{

        //获取速度
        speed_R =  encoder_get_count(TIM6_ENCODER);encoder_clear_count(TIM6_ENCODER);//
        speed_L = -encoder_get_count(TIM5_ENCODER);encoder_clear_count(TIM5_ENCODER);//
        if (!motor_pwm_ready)
        {
            Speed_OUT_L = 0;
            Speed_OUT_R = 0;
            fuya_pwm = 0;
            return;  /* Keep sampling, without running PI or touching EN. */
        }


        //内减外不加方案
    //    Speed_OUT_R = PID_motor_R(&SPEED_R,speed_R ,Speed_Goal+imu_Err);
    //    Speed_OUT_L = PID_motor_L(&SPEED_L,speed_L ,Speed_Goal-imu_Err);
        //自己加的
//        if(flag_motor==0)
//                {
//                    pwm_set_duty(MOTOR_L, 0);gpio_set_level(DIR_L,0);
//                    pwm_set_duty(MOTOR_R, 0);gpio_set_level(DIR_R,0);
//                }
//        else
//        {
//            fuya_pwm=800;
            //Motor_Protect();
        if(SystemData.Stop == 1)
              {
                  fuya_pwm=0;

                  Speed_OUT_R = PID_motor_R(&SPEED_R,speed_R ,0);
                  Speed_OUT_L = PID_motor_L(&SPEED_L,speed_L ,0);

                                          if(Speed_OUT_L>=0){pwm_set_duty(MOTOR_L, Speed_OUT_L);gpio_set_level(DIR_L,1);}
                                          else              {pwm_set_duty(MOTOR_L,-Speed_OUT_L);gpio_set_level(DIR_L,0);}
                                          if(Speed_OUT_R>=0){pwm_set_duty(MOTOR_R, Speed_OUT_R);gpio_set_level(DIR_R,1);}
                                          else              {pwm_set_duty(MOTOR_R,-Speed_OUT_R);gpio_set_level(DIR_R,0);}





              }
              else
              {
                  const float speed_scale = TRACK_SPEED_PERCENT / 100.0f;
                  const float base = Speed_Goal;
                  const float error = Err;
                  float right = base;
                  float left = base;
                  if (error > 0)
                  {
                      right -= error;
                      left += xishu * error;
                  }
                  else if (error < 0)
                  {
                      right -= xishu * error;
                      left += error;
                  }
                  Speed_OUT_R = PID_motor_R(&SPEED_R, speed_R, right * speed_scale);
                  Speed_OUT_L = PID_motor_L(&SPEED_L, speed_L, left * speed_scale);
                  fuya_pwm=850;
                  //Motor_Protect();


                         if(Speed_OUT_L>=0){pwm_set_duty(MOTOR_L, Speed_OUT_L);gpio_set_level(DIR_L,1);}
                         else              {pwm_set_duty(MOTOR_L,-Speed_OUT_L);gpio_set_level(DIR_L,0);}
                         if(Speed_OUT_R>=0){pwm_set_duty(MOTOR_R, Speed_OUT_R);gpio_set_level(DIR_R,1);}
                         else              {pwm_set_duty(MOTOR_R,-Speed_OUT_R);gpio_set_level(DIR_R,0 );}
              }

//        }

}


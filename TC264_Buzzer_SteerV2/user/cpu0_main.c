/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
#define WIFI_SSID_TEST          "HUAWEI"
#define WIFI_PASSWORD_TEST      "UnderTaker"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.127.89"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8080"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666
void my_init(void)
{


    gpio_init(DIR_L, GPO, 1, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, 1, GPO_PUSH_PULL);



    ips200_init(IPS200_TYPE_SPI);
    system_delay_ms(1000); // Keep the on-car startup timing before camera and servo power-up activity.
    mt9v03x_init ();   // 摄像头初始化
    servo_init(); // P33_9: steering servo, centered at startup
    buzzer_init(); // P11.11 active buzzer, off before interrupts start.



    imu660ra_init();





    encoder_dir_init(TIM5_ENCODER,TIM5_ENCODER_CH1_P10_3 ,TIM5_ENCODER_CH2_P10_1);//编码器初始化带方向
    encoder_dir_init(TIM6_ENCODER,TIM6_ENCODER_CH1_P20_3 ,TIM6_ENCODER_CH2_P20_0);

    pwm_init(ATOM0_CH0_P21_2,50,0);//负压风扇1初始化
    pwm_init(ATOM0_CH1_P21_3,50,0);//负压风扇2初始化

    gpio_init(KEY_1,GPI,0,GPI_PULL_UP);//按键初始化
    gpio_init(KEY_2,GPI,0,GPI_PULL_UP);//按键初始化
    gpio_init(KEY_3,GPI,0,GPI_PULL_UP);//按键初始化
    gpio_init(KEY_4,GPI,0,GPI_PULL_UP);//按键初始化


    // Enable periodic control after all controlled peripherals are ready.
    pit_init(CCU60_CH0,5000);       //  5ms中断 位置式pid控制
    pit_init(CCU60_CH1,5000);       //  5ms中断 获取角速度
    pit_init(CCU61_CH1,5000);        // 5ms中断 电机控制
}

void Data_Settings(void)           //参数赋值
{
  //图像参数
  //adcsum = 0;
  ImageStatus.MiddleLine = 39;//中线
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.7;  //待定自动补线斜率
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;

  ImageStatus.TowPoint =26;           //前瞻  ///
  ImageStatus.Threshold_static = 70;   //静态阈值  40-80
  ImageStatus.Threshold_detach = 180;  //阳光算法  亮斑分离140-220
  ImageScanInterval = 2;               //扫边范围    上一行的边界+-ImageScanInterval
  ImageScanInterval_Cross = 2;         //十字扫线范围
 // ImageStatus.variance = 26;           //直道方差阈值
  ImageStatus.variance_acc = 25;       //直道检测
//  SystemData.outbent_acc  =  5;
 // SystemData.clrcle_num=0;
  //ImageStatus.newblue_flag=0;
   SystemData.Stop = 1;                 //停车
  //BlueTooth_Flag=1;

  /**位置式pid参数**/
  // 重中之重： 电池电压低于12V 去充电 否则参数都是不对的
  // 方向环可以先调P 发现P已经转弯接近内切的时候，再去加D
  //SteerPIDdata.P = Read_Flash_float(0);  //16 原来2.2 5.4大了 3.2小了 4.2大了一点点 3.9      原来2.9   100hz参数      200hz参数:9.0 9.5 10.5 11.5 12.0 13.0
  //  11.0  比较适合的值：10.5
 // SteerPIDdata.I = 0.0;
 // SteerPIDdata.D = Read_Flash_float(6);  //原来7.2                       9.0 10.0   目前最好9.5                               原来6.8 测试200hz参数:29.0
  //200hz D：32小了 35也有点小大 39大 37大了
}
// **************************** 代码区域 ****************************
uint8 LK_STATE=0;
int core0_main(void)
{
    uint8 key4_was_pressed = 0;
    uint8 start_requested = 0;
    clock_init();                   // 获取时钟频率<务必保留>
    gpio_init(MOTOR_L_EN_GPIO, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_EN_GPIO, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(IPS200_BLk_PIN_SPI, GPO, GPIO_HIGH, GPO_PUSH_PULL); // Backlight before peripheral setup.
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    Data_Settings(); // Set the stopped state before enabling control interrupts.
    my_init();
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    key4_was_pressed = (gpio_get_level(KEY_4) == 0); // Require a fresh press after boot.
    while (TRUE)
    {
//        gpio_set_level(DIR_R,0);
        //进入菜单
        if(gpio_get_level(KEY_1)==0)
        {
            //while(gpio_get_level(KEY_1)==0);
            system_delay_ms(100);
            Menu_main();
        }
        // KEY4每次按下只启动一次；长按不能在终点自动重新启动。
        if(gpio_get_level(KEY_4) != 0)
        {
            key4_was_pressed = 0;
        }
        else if(!key4_was_pressed)
        {
            key4_was_pressed = 1;
            if(SystemData.Stop == 1 && !start_requested)
            {
                system_delay_ms(100);
                if(gpio_get_level(KEY_4) == 0)
                {
                    Var_speed_start = 0;
                    lap_counter_request_start();
                    start_requested = 1;
                }
            }
        }
        // 等CPU1完成计圈复位和上一帧速度处理，再从零缓启动。
        if(start_requested && !lap_counter_start_pending())
        {
            Speed_Goal = 0;
            LK_STATE = 0;
            start_requested = 0;
            SystemData.Stop = 0;
            motor_enable_on_first_start();
        }
        if(LK_STATE==0&&SystemData.Stop == 0)
        {
            int16 startup_speed = Speed_Goal + 25;
            if(startup_speed > Speed_Goal_min)
                startup_speed = Speed_Goal_min;
            Speed_Goal = startup_speed;
            system_delay_ms(50);
            if(Speed_Goal>=Speed_Goal_min)
            {
                LK_STATE=1;
                Var_speed_start=1;
            }
        }
        // 第二次返回起终点后，沿用原来的800ms过线停车延时。
        // CPU1确认复位前，不消费上一轮留下的终点标志。
        if(SystemData.Stop == 0 && !lap_counter_start_pending() && banmaxian_on == 1)
        {
            system_delay_ms(800);
            Var_speed_start = 0;
            Speed_Goal = 0;
            SystemData.Stop = 1;
            key4_was_pressed = 1;
        }
        //显示图像
       if(SystemData.Stop == 1)
       {
         ips200();
       }
      // pwm_set_duty(ATOM0_CH0_P21_2, fuya_pwm);
      // pwm_set_duty(ATOM0_CH1_P21_3, fuya_pwm);
    }
}
#pragma section all restore

#include "Menu.h"

uint16_t cursor=0;//光标
uint8_t page_select=0;//页面

/*****************PID参数相关菜单**********************/
void set_P(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            ERROR.Kp+=0.05;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            ERROR.Kp-=0.05;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,ERROR.Kp,3,2);
        ips200_show_string(2,22, "KP");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_D(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            ERROR.Kd+=0.05;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            ERROR.Kd-=0.05;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,ERROR.Kd,3,2);
        ips200_show_string(2,22, "KD");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_dynamic_P_A(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            P_A+=0.01;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            P_A-=0.01;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,P_A,3,2);
        ips200_show_string(2,22, "P_A");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_dynamic_P_B(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            P_B+=0.1;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            P_B-=0.1;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,P_B,3,2);
        ips200_show_string(2,22, "P_B");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_dynamic_D(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            D_coe+=0.1;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            D_coe-=0.1;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,D_coe,3,2);
        ips200_show_string(2,22, "D_coe");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_Kp_TWO(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            kp_two+=0.01;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            kp_two-=0.01;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,kp_two,3,2);
        ips200_show_string(2,22, "kp_two");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_pid_mode(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            pid_mode++;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            pid_mode--;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,pid_mode,3,2);
        ips200_show_string(2,22, "pid_mode");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void set_GK_d(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            GKd+=0.05;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            GKd-=0.05;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,GKd,3,2);
        ips200_show_string(2,22, "GKd");//变量名
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void Menu_PID(void)
{
    ips200_clear();
    cursor=22;
    page_select=1;
    while(1)
    {
        //光标下移
        if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            cursor+=20;
            page_select++;
            //限幅
            if(cursor>=182)     cursor=182;
            if(page_select>=15) page_select=15;
            ips200_clear();
        }
        //光标上移
        if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            cursor-=20;
            page_select--;
            //限幅
            if(cursor<=22)     cursor=22;
            if(page_select<=1) page_select=1;
            ips200_clear();
        }
        //进入选项
        if(gpio_get_level(KEY_4)==0)
        {
            while(gpio_get_level(KEY_4)==0);
            ips200_clear();
            if(page_select==1)
            {
                break;
            }
            else if(page_select==2)
            {
                set_P();
            }
            else if(page_select==3)
            {
                set_D();
            }
            else if(page_select==4)
            {
                set_dynamic_P_A();
            }
            else if(page_select==5)
            {
                set_dynamic_P_B();
            }
            else if(page_select==6)
            {
                set_dynamic_D();
            }
            else if(page_select==7)
            {
                set_Kp_TWO();
            }
            else if(page_select==8)
            {
                set_pid_mode();
            }
            else if(page_select==9)
            {
                set_GK_d();
            }
        }
        ips200_show_string(22,2,"PID MENU");//显示标题
        ips200_show_string(0,cursor,">");//显示光标
        ips200_show_string(8,22,"break");//1
        ips200_show_string(8,42,"set_P");//2
        ips200_show_string(8,62,"set_D");//3
        ips200_show_string(8,82,"set_dynamic_P_A");//4
        ips200_show_string(8,102,"set_dynamic_P_B");//5
        ips200_show_string(8,122,"set_dynamic_D");//6
        ips200_show_string(8,142,"set_kp_two");//7
        ips200_show_string(8,162,"set pid mode");//8
        ips200_show_string(8,182,"set GKd");//9
    }
}
/*****************图像相关菜单**********************/
uint8_t road_pos=0;
void get_road_wide(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            road_pos++;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            road_pos--;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_float(2,2,road_pos,3,2);
        ips200_show_int(2,22,ImageDeal[road_pos].RightBorder-ImageDeal[road_pos].LeftBorder,3);
        ips200_show_string(2,42, "road pos");//变量名
        ips200_show_string(2,62, "+     ->key1");//数值加
        ips200_show_string(2,82, "-     ->key2");//数值减
        ips200_show_string(2,102, "break ->key3");//退出
    }
}
void Menu_image(void)
{
    ips200_clear();
    cursor=22;
    page_select=1;
    while(1)
    {
        //光标下移
        if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            cursor+=20;
            page_select++;
            //限幅
            if(cursor>=152)     cursor=152;
            if(page_select>=15) page_select=15;
            ips200_clear();
        }
        //光标上移
        if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            cursor-=20;
            page_select--;
            //限幅
            if(cursor<=22)     cursor=22;
            if(page_select<=1) page_select=1;
            ips200_clear();
        }
        //进入选项
        if(gpio_get_level(KEY_4)==0)
        {
            while(gpio_get_level(KEY_4)==0);
            ips200_clear();
            if(page_select==1)
            {
                break;
            }
            if(page_select==2)
            {
                get_road_wide();
            }
        }
        ips200_show_string(22,2,"image MENU");//显示标题
        ips200_show_string(0,cursor,">");//显示光标
        ips200_show_string(8,22,"break");//1
        ips200_show_string(8,42,"Road wide");//2
    }
}
/*****************速度相关菜单**********************/
void my_set_speed(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            Speed_Goal+=5;
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            Speed_Goal-=5;
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_int(2,2,Speed_Goal,3);
        ips200_show_string(2,22, "Speed_Goal");//显示标题
        ips200_show_string(2,42, "+      ->key1");//数值加
        ips200_show_string(2,62,"-      ->key2");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}

void set_yuanhuan(void)
{
    while(1)
    {
        if(gpio_get_level(KEY_1)==0)
        {
            while(gpio_get_level(KEY_1)==0);
            YuanHuan=1;//左圆环
        }
        else if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            YuanHuan=2;//右圆环
        }
        else if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            ips200_clear();
            break;
        }
        ips200_show_int(2,2,YuanHuan,3);
        ips200_show_string(2,22, "YuanHuan");//显示标题
        ips200_show_string(2,42, "key1      ->zuo");//数值加
        ips200_show_string(2,62,"key2      ->you");//数值减
        ips200_show_string(2,82,"break  ->key3");//退出
    }
}
void Menu_speed(void)
{
    ips200_clear();
    cursor=22;
    page_select=1;
    while(1)
    {
        //光标下移
        if(gpio_get_level(KEY_3)==0)
        {
            while(gpio_get_level(KEY_3)==0);
            cursor+=20;
            page_select++;
            //限幅
            if(cursor>=152)     cursor=152;
            if(page_select>=15) page_select=15;
            ips200_clear();
        }
        //光标上移
        if(gpio_get_level(KEY_2)==0)
        {
            while(gpio_get_level(KEY_2)==0);
            cursor-=20;
            page_select--;
            //限幅
            if(cursor<=22)     cursor=22;
            if(page_select<=1) page_select=1;
            ips200_clear();
        }
        //进入选项
        if(gpio_get_level(KEY_4)==0)
        {
            while(gpio_get_level(KEY_4)==0);
            ips200_clear();
            if(page_select==1)
            {
                break;
            }
            else if(page_select==2)
            {
                my_set_speed();
            }
        }
        ips200_show_string(22,2,"speed MENU");//显示标题
        ips200_show_string(0,cursor,">");//显示光标
        ips200_show_string(8,22,"break");//1
        ips200_show_string(8,42,"set speed");//2
    }
}

void Menu_yuanhuan(void)
{
    ips200_clear();
       cursor=22;
       page_select=1;
       while(1)
       {
           //光标下移
           if(gpio_get_level(KEY_3)==0)
           {
               while(gpio_get_level(KEY_3)==0);
               cursor+=20;
               page_select++;
               //限幅
               if(cursor>=152)     cursor=152;
               if(page_select>=15) page_select=15;
               ips200_clear();
           }
           //光标上移
           if(gpio_get_level(KEY_2)==0)
           {
               while(gpio_get_level(KEY_2)==0);
               cursor-=20;
               page_select--;
               //限幅
               if(cursor<=22)     cursor=22;
               if(page_select<=1) page_select=1;
               ips200_clear();
           }
           //进入选项
           if(gpio_get_level(KEY_4)==0)
           {
               while(gpio_get_level(KEY_4)==0);
               ips200_clear();
               if(page_select==1)
               {
                   break;
               }
               else if(page_select==2)
               {
                   set_yuanhuan();
               }
           }
           ips200_show_string(22,2,"yuanhuan MENU");//显示标题
           ips200_show_string(0,cursor,">");//显示光标
           ips200_show_string(8,22,"break");//1
           ips200_show_string(8,42,"set yuanhuan");//2
       }
}
/*****************主菜单**********************/
void Menu_main(void)
{
    ips200_clear();
    cursor=22;
    page_select=1;

    while (1)
    {
        //ips200_full(RGB565_BLACK);
        //光标下移
        if (gpio_get_level(KEY_3) == 0)
        {
            while (gpio_get_level(KEY_3) == 0)
                ;
            cursor += 20;
            page_select++;
            //限幅
            if (cursor >= 152)
                cursor = 152;
            if (page_select >= 15)
                page_select = 15;
            ips200_clear();
        }

        //光标上移
        //ips200_show_int(0,0,gpio_get_level(KEY_2),2);
        if(gpio_get_level(KEY_2)==0)
         {
         while(gpio_get_level(KEY_2)==0);
         cursor-=20;
         page_select--;
         //限幅
         if(cursor<=12)     cursor=12;
         if(page_select<=1) page_select=1;
         ips200_clear();
         }
        //进入选项
         if(gpio_get_level(KEY_4)==0)
         {
         while(gpio_get_level(KEY_4)==0);
         ips200_clear();
         if(page_select==1)
         {
         break;
         }
         else if(page_select==2)
         {
         Menu_PID();
         }
         else if(page_select==3)
         {
         Menu_image();
         }
         else if(page_select==4)
         {
         Menu_speed();
         }
         else if(page_select==5)
         {
         Menu_yuanhuan();
         }

         }
        /* ips200_show_string(22,2,"MENU");//显示标题
         ips200_show_string(0,cursor,">");//显示光标
         ips200_show_string(8,12,"break");//1
         ips200_show_string(8,22,"Menu_PID");//2
         ips200_show_string(8,32,"Menu_image");//3
         ips200_show_string(8,42,"Menu_speed");//4
         ips200_show_string(8,52,"Menu_yuanhuan");//5*/
        ips200_show_string(42, 2, "MENU");       //显示标题
        ips200_show_string(10, cursor, ">");       //显示光标
        ips200_show_string(28, 22, "break");       //1
        ips200_show_string(28, 42, "Menu_PID");       //2
        ips200_show_string(28, 62, "Menu_image");       //3
        ips200_show_string(28, 82, "Menu_speed");       //4
        ips200_show_string(28, 102, "Menu_yuanhuan");       //5

    }
}

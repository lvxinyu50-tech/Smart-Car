#include "system.h"


int16 KeyNum;
int image_flag;
void Key_Scan(void)
{
    static int16 laststate = 0,nowstate = 0;
    laststate = nowstate;
    if      (!gpio_get_level(KEY1))nowstate = 1;
    else if (!gpio_get_level(KEY2))nowstate = 2;
    else if (!gpio_get_level(KEY3))nowstate = 3;
    else if (!gpio_get_level(KEY4))nowstate = 4;
    else                           nowstate = 0;

    if      (laststate == 1 && nowstate == 0)KeyNum = 1;
    else if (laststate == 2 && nowstate == 0)KeyNum = 2;
    else if (laststate == 3 && nowstate == 0)KeyNum = 3;
    else if (laststate == 4 && nowstate == 0)KeyNum = 4;

    switch (KeyNum)
    {
        case 1: image_flag ^= 1;KeyNum = 0;break;
        case 2: KeyNum = 0;break;
        case 3: KeyNum = 0;break;
        case 4: KeyNum = 0;break;
        default:break;
    }
}


void ips200(void)
{
//    if(image_flag)
//    ips200_show_gray_image(0,0,Image_Use[0],LCDW,LCDH,LCDW,LCDH,0);
//    else





    ips200_show_gray_image(0,0,Pixle[0],LCDW,LCDH,LCDW,LCDH,1);
    ips200_draw_point(Point_Xsite, Point_Ysite, RGB565_CYAN);



    for(int i=ImageStatus.OFFLine;i<=59;i+=1)
    {
        ips200_draw_point((int)ImageDeal[i].LeftBorder,i,RGB565_RED);
        ips200_draw_point((int)ImageDeal[i].RightBorder,i,RGB565_RED);
        ips200_draw_point((int)ImageDeal[i].Center,i,RGB565_RED);
    }
//    for(uint16 i=0;i<=79;i++)
//    {
//        ips200_draw_point(i,ImageStatus.OFFLine,RGB565_RED);
////        ips200_draw_point(i,ImageStatus.TowP_True-5,RGB565_CYAN);
////        ips200_draw_point(i,ImageStatus.TowP_True+5,RGB565_CYAN);//
//    }
//图像原始偏差
    ips200_show_string(0,95,"yh");
    ips200_show_float(40,95,YuanHuan,2,2);
    // 已完成的整圈数（起跑时的斑马线不计圈）。
    ips200_show_string(0,115,"lap");
    ips200_show_int(40,115,lap_count,1);
    ips200_show_string(50,115,"/");
    ips200_show_int(60,115,LAP_TARGET_COUNT,1);

     ips200_show_string(0,155,"round");//圆环
     ips200_show_float(50,155,ImageFlag.image_element_rings_flag,2,2);


//     ips200_show_string(0,125,"zxpd");
//     ips200_show_float(30,125,zxpd,2,2);

     ips200_show_string(0,135,"you");//圆环
     ips200_show_float(50,135,yuoyuan1,2,2);

     ips200_show_string(0,175,"asda");//圆环
        ips200_show_float(50,175,asda,2,2);

        ips200_show_string(0,195,"qwer");//圆环
               ips200_show_float(50,195,qwer,2,2);






//显示速度
    ips200_show_string(131,0,"L");
    ips200_show_int   (152,0,speed_L,3);

    ips200_show_string(131,20,"R");
    ips200_show_int   (152,20,speed_R,3);






//求出的误差
    ips200_show_string(131,40,"ER");
    ips200_show_int   (152,40,Err,3);

//目标速度
//    ips200_show_string(81,30,"x25");
//    ips200_show_int   (102,30,f1,3);
//陀螺仪
    ips200_show_string(131,60,"z");
    ips200_show_int(152,60,imu_z,3);
//
//    ips200_show_string(81,50,"r");
//    ips200_show_int(102,50,RingType.ring_flag,1);
//
//    ips200_show_string(81,60,"rLR");
//    ips200_show_int(102,60,RingType.ring_LR,1);
//左边
    ips200_show_string(131,80,"L_L");
    ips200_show_int(180,80,ImageStatus.Left_Line,2);
//右边
    ips200_show_string(131,100,"R_L");
    ips200_show_int(180,100,ImageStatus.Right_Line,2);
//双边丢线数
    ips200_show_string(131,120,"W_L");
    ips200_show_int(180,120,ImageStatus.WhiteLine,2);
//
    ips200_show_string(131,140,"vac");
    ips200_show_int(180,140,variance_acc,2);
//图像顶边
    ips200_show_string(131,160,"OFF");
    ips200_show_int(180,160,ImageStatus.OFFLine,2);
//
    ips200_show_string(131,180,"Stop");
    ips200_show_int(180,180,SystemData.Stop,2);
//    ips200_show_string(81,130,"acc");
//    ips200_show_int(102,130,acc,2);


}


//void upper_computer(void)
//{
////    if(image_flag)
////    ips200_show_gray_image(0,0,Image_Use[0],LCDW,LCDH,LCDW,LCDH,0);
////    else
//
//    for(int i=ImageStatus.OFFLine;i<=59;i+=1)
//        {
//
//            sendimgAndLine_type(RGB565_RED,(int)ImageDeal[i].LeftBorder,i,4);
//            sendimgAndLine_type(RGB565_BLUE,(int)ImageDeal[i].RightBorder,i,4);
//            sendimgAndLine_type(RGB565_YELLOW,(int)ImageDeal[i].Center,i,4);
//
//    //        sendimgAndLine_type((int)ImageDeal[i].LeftBorder,i,RGB565_RED);
//    //        sendimgAndLine_type((int)ImageDeal[i].RightBorder,i,RGB565_RED);
//    //        sendimgAndLine_type((int)ImageDeal[i].Center,i,RGB565_BLACK);
//        }
//
//    sendimg(Pixle[0],MT9V03X_W,MT9V03X_H);
//
//    for(uint16 i=0;i<=79;i++)
//    {
////        sendimg(i,ImageStatus.OFFLine,RGB565_RED);
//        sendimgAndLine_type(RGB565_PURPLE,ImageStatus.OFFLine,i,1);
////        ips200_draw_point(i,ImageStatus.TowP_True-5,RGB565_CYAN);
////        ips200_draw_point(i,ImageStatus.TowP_True+5,RGB565_CYAN);//
//    }
////图像原始偏差
////    put_string("dt",2);
////    ips200_show_float(20,95,ImageStatus.Det_True-39,2,2);
//    put_float(1,ImageStatus.Det_True-39);
////斑马线标志位banmaxian_on
////    ips200_show_string(0,105,"bnama");
////    ips200_show_float(30,105,banmaxian_on,2,2);
//    put_float(2,banmaxian_on);
//
////    ips200_show_string(0,115,"yuan");
////    ips200_show_float(30,115,ImageFlag.image_element_rings_flag,2,2);
//    put_float(3,ImageFlag.image_element_rings_flag);
//
//
////显示速度
////    ips200_show_string(81,0,"L");
////    ips200_show_int   (102,0,speed_L,3);
//    put_float(4,speed_L);
//
////    ips200_show_string(81,10,"R");
////    ips200_show_int   (102,10,speed_R,3);
//    put_float(5,speed_R);
////求出的误差
////    ips200_show_string(81,20,"ER");
////    ips200_show_int   (102,20,Err,3);
//    put_float(6,Err);
////目标速度
////    ips200_show_string(81,30,"SG");
////    ips200_show_int   (102,30,Speed_Goal,3);
//    put_float(7,Speed_Goal);
////陀螺仪
////    ips200_show_string(81,40,"z");
////    ips200_show_int(102,40,imu_z,3);
//    put_float(8,imu_z);
////
////    ips200_show_string(81,50,"r");
////    ips200_show_int(102,50,RingType.ring_flag,1);
////
////    ips200_show_string(81,60,"rLR");
////    ips200_show_int(102,60,RingType.ring_LR,1);
////左边
////    ips200_show_string(81,70,"L_L");
////    ips200_show_int(102,70,ImageStatus.Left_Line,2);
//    put_float(9,ImageStatus.Left_Line);
////右边
////    ips200_show_string(81,80,"R_L");
////    ips200_show_int(102,80,ImageStatus.Right_Line,2);
//    put_float(10,ImageStatus.Right_Line);
////双边丢线数
////    ips200_show_string(81,90,"W_L");
////    ips200_show_int(102,90,ImageStatus.WhiteLine,2);
//    put_float(11,ImageStatus.WhiteLine);
////
////    ips200_show_string(81,100,"vac");
////    ips200_show_int(102,100,variance_acc,2);
//    put_float(12,variance_acc);
////图像顶边
////    ips200_show_string(81,110,"OFF");
////    ips200_show_int(102,110,ImageStatus.OFFLine,2);
//    put_float(13,ImageStatus.OFFLine);
////
////    ips200_show_string(81,120,"Stop");
////    ips200_show_int(105,120,SystemData.Stop,2);
//    put_float(14,SystemData.Stop);
////    ips200_show_string(81,130,"acc");
////    ips200_show_int(102,130,acc,2);
//
//
//}



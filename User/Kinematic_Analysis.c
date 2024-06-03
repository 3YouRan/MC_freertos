//
// Created by 陈瑜 on 24-6-3.
//

#include "Kinematic_Analysis.h"
extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;

/**************************************************************************
麦克纳姆轮逆运动学模型
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    Target_Speed_A=-(Vx-Vy-V_angle*RxPLUSRy);//左前
    Target_Speed_B=-(Vx+Vy+V_angle*RxPLUSRy);//右前
    Target_Speed_C=-(Vx+Vy-V_angle*RxPLUSRy);//左后
    Target_Speed_D=-(Vx-Vy+V_angle*RxPLUSRy);//右后
}
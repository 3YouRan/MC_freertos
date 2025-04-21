//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"

Base_status_t Base_status;

uint8_t mode_flag = 1;
uint8_t direction = FORWARD;
uint8_t delay_times = 0;
uint8_t delay_flag = 0;

void Base_Control(void *argument){

    while(1){
        if(delay_flag==0) {
            delay_times++;
        }
        if(delay_times>10){
            delay_flag = 1;
            delay_times = 0;
        }
        if(mode_flag==RC_MODE&&delay_flag==1) {//遥控模式，遥控器控制
            //速度控制
            if((L_TICK[1]+L_TICK[0])>500){//未连接遥控器时，默认停止
                Base_status.vx = 0;
                Base_status.vy = 0;

            }else if(abs((L_TICK[0]-128)+(L_TICK[1]-128))>20){
                Base_status.vx = (L_TICK[0]-128)*5;
                Base_status.vy = (L_TICK[1]-128)*5;

            }else {
                Base_status.vx = 0;
                Base_status.vy = 0;
            }

            Kinematic_Analysis(Base_status.vx,Base_status.vy,Base_status.omega);
        }
        else if(mode_flag==SLAVE_MODE&&delay_flag==1) {//slave模式，上位机串口控制

        }

        vTaskDelay(20);
    }
}

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
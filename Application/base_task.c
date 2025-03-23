//
// Created by ³Âè¤ on 25-3-22.
//
#include "all.h"

float Vx;
float Vy;
float angle_speed;
uint8_t mode_flag = 1;
uint8_t direction = FORWARD;
uint8_t delay_times = 0;
uint8_t delay_flag = 0;
float servo1_angle = 0;
float servo2_angle = 0;
void Base_Control(void *argument){

    while(1){
        if(delay_flag==0) {
            delay_times++;
        }
        if(delay_times>10){
            delay_flag = 1;
            delay_times = 0;
        }
        if(mode_flag==1&&delay_flag==1) {//Ò£¿ØÄ£Ê½£¬Ò£¿ØÆ÷¿ØÖÆ
            //ËÙ¶È¿ØÖÆ
            if((L_TICK[1]+L_TICK[0])>500){//Î´Á¬½ÓÒ£¿ØÆ÷Ê±£¬Ä¬ÈÏÍ£Ö¹
                Vx = 0;
                Vy = 0;

            }else if(abs((L_TICK[0]-128)+(L_TICK[1]-128))>20){
                Vy = (L_TICK[0]-128)*5;
                Vx = (L_TICK[1]-128)*5;

            }else {
                Vx = 0;
                Vy = 0;
            }




            Kinematic_Analysis(Vx,Vy,-angle_speed);
        }

        vTaskDelay(20);
    }
}
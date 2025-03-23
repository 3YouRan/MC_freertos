//
// Created by 陈瑜 on 25-3-23.
//

#include "all.h"
//设置舵机角度:0-270度 0:脉宽0.5ms,270:脉宽2.5ms PWM周期20ms
void servo1_set(float angle) {
    if(angle<0)//限制角度范围
        angle=0;
    if(angle>270)
        angle=270;

    float width=angle/270.0f;//角度转脉宽比例
    width=width*(PW_MAX-PW_MIN)+PW_MIN;//脉宽比例转脉宽值

    SERVO1_SET((uint16_t)width);

}
//设置舵机角度:0-180度 0:脉宽0.5ms,180:脉宽约1.83ms PWM周期20ms
void servo2_set(float angle) {
    if(angle<0)//限制角度范围
        angle=0;
    if(angle>180)
        angle=180;

    float width=angle/270.0f;//角度转脉宽比例
    width=width*(PW_MAX-PW_MIN)+PW_MIN;//脉宽比例转脉宽值

    SERVO2_SET((uint16_t)width);

}
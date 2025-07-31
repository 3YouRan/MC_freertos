//
// Created by 陈瑜 on 25-3-23.
//

#include "all.h"
//设置舵机角度:0-270度 0:脉宽0.5ms,180:脉宽2.5ms PWM周期20ms
uint16_t Ang_to_Pulse_Num(float angle) {
    uint16_t pulse_num = (angle/360.0f)*200.0f*32.0f; // 角度转脉冲数

    return pulse_num;
}
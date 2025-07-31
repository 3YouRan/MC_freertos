//
// Created by 陈瑜 on 24-6-2.
//

#ifndef MC_PROJ_PID_CONTROL_H
#define MC_PROJ_PID_CONTROL_H
#include "stm32f4xx.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "timers.h"


#define FORWARD 5
#define LEFT 8
#define BACK 7
#define RIGHT 6

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值
    float output,maxOutput;
    float deadZone; //死区
}PID;



void PID_Init();
void Base_Control(void *argument);
float INC_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
float FULL_PID_Realize(PID* pid,float target,float feedback);//一次PID计算


#endif //MC_PROJ_PID_CONTROL_H

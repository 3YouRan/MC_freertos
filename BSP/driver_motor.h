//
// Created by 陈瑜 on 24-6-2.
//

#ifndef MC_PROJ_DRIVER_MOTOR_H
#define MC_PROJ_DRIVER_MOTOR_H
#include "main.h"
#include "tim.h"

#define ENCODER1  &htim2
#define ENCODER2  &htim3
#define ENCODER3  &htim4
#define ENCODER4  &htim5

#define PWMA1_SET(Val) __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, Val); //Val(0~1000)
#define PWMA2_SET(Val) __HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Val); //Val(0~1000)
#define PWMB1_SET(Val) __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, Val);
#define PWMB2_SET(Val) __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, Val);
#define PWMC1_SET(Val) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Val);
#define PWMC2_SET(Val) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Val);
#define PWMD1_SET(Val) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Val); //Val(0~1000)
#define PWMD2_SET(Val) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Val); //Val(0~1000)

typedef struct motor{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
    float TargetSpeed;   //目标转速
    float TargetSpeed_now; //当前目标转速
}motor;


void motor_init();
void motorA_run(int speed);
void motorB_run(int speed);
void motorC_run(int speed);
void motorD_run(int speed);




#endif //MC_PROJ_DRIVER_MOTOR_H

//
// Created by 陈瑜 on 24-6-2.
//

#include "driver_motor.h"
motor motorA, motorB, motorC, motorD;
void motor_init(){
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);

    motorA.totalCount=0;
    motorA.speed=0;
    motorA.direct=0;
    motorA.lastCount=0;
    motorA.overflowNum=0;

    motorB.totalCount=0;
    motorB.speed=0;
    motorB.direct=0;
    motorB.lastCount=0;
    motorB.overflowNum=0;

    motorC.totalCount=0;
    motorC.speed=0;
    motorC.direct=0;
    motorC.lastCount=0;
    motorC.overflowNum=0;

    motorD.totalCount=0;
    motorD.speed=0;
    motorD.direct=0;
    motorD.lastCount=0;
    motorD.overflowNum=0;
}

void motorA_run(int speed){
    if (speed>0){
        PWMA1_SET(0);
        PWMA2_SET(speed);
    }
    else if (speed<0){
        PWMA1_SET(-speed);
        PWMA2_SET(0);
    }else{
        PWMA1_SET(0);
        PWMA2_SET(0);
    }
}
void motorB_run(int speed){

    if (speed>0){
        PWMB1_SET(speed);
        PWMB2_SET(0);

    }
    else if (speed<0){
        PWMB1_SET(0);
        PWMB2_SET(-speed);
    }else{
        PWMB1_SET(0);
        PWMB2_SET(0);
    }

}
void motorC_run(int speed){
    if (speed>0){
        PWMC1_SET(speed);
        PWMC2_SET(0);


    }
    else if (speed<0){
        PWMC1_SET(0);
        PWMC2_SET(-speed);
    }else{
        PWMC1_SET(0);
        PWMC2_SET(0);
    }
}
void motorD_run(int speed){
    if (speed>0){
        PWMD1_SET(0);
        PWMD2_SET(speed);
    }
    else if (speed<0){
       PWMD1_SET(-speed);
        PWMD2_SET(0);
    }else{
        PWMD1_SET(0);
        PWMD2_SET(0);
    }
}



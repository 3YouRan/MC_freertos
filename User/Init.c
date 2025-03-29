//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"
void my_init() {
    //电源电压ADC采样初始化
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    //PS2手柄初始化
    PS2_SetInit();
    //定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim6);
    //电机控制PWM初始化
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    //舵机控制PWM初始化
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
    //串口重定向初始化
    RetargetInit(&huart2);
    //UART接收中断初始化
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 启动UART接收中断
//    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // 启动UART接收中断

    HAL_UART_Receive_DMA(&huart6,&rx_byte,1);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuffer_UP, 1);   // 启动UART接收中断
    //电机初始化
    motor_init();
    //PID初始化
    PID_Init();

    //OLED初始化
    OLED_Init();
    //滤波器初始化
//    initialize_LowPassFilter(&filter_yaw, 0.85f);

}
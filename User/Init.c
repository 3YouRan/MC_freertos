//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"
void my_init() {

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    PS2_SetInit();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    RetargetInit(&huart2);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 启动UART接收中断
    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // 启动UART接收中断

    motor_init();
    PID_Init();
    Cmd_03();//唤醒传感??
    Cmd_12(5, 255, 0,  0, 3, 100, 2, 4, 9, 0x40);//设置IM600设备参数
    Cmd_05();// 归零IM600Z轴姿态角数据，以小车复位时的姿???角为角??0??
    Cmd_19();// ??启数据主动上??
    OLED_Init();
}
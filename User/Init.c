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
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    //串口重定向初始化
    RetargetInit(&huart2);
    //UART接收中断初始化
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 启动UART接收中断
    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // 启动UART接收中断
    //电机初始化
    motor_init();
    //PID初始化
    PID_Init();
    //IM600初始化
    Cmd_03();//唤醒传感??
    Cmd_12(5, 255, 0,  0, 3, 100, 2, 4, 9, 0x40);//设置IM600设备参数
    Cmd_05();// 归零IM600Z轴姿态角数据，以小车复位时的姿???角为角??0??
    Cmd_19();// ??启数据主动上??
    //OLED初始化
    OLED_Init();


}
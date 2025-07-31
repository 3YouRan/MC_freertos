//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"

void my_init() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
    //电机控制PWM初始化
    HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4);
    //串口重定向初始化
    RetargetInit(&huart2);
    //UART接收中断初始化
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 启动UART接收中断
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DataBuff_UP, 200);   // 启动UART接收中断

    Set_Target_UartInit();
    //OLED初始化
    OLED_Init();
    Set_Target_UartInit();

}
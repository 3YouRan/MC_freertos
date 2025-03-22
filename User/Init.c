//
// Created by ��� on 25-3-22.
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
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // ����UART�����ж�
    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // ����UART�����ж�

    motor_init();
    PID_Init();
    Cmd_03();//���Ѵ���??
    Cmd_12(5, 255, 0,  0, 3, 100, 2, 4, 9, 0x40);//����IM600�豸����
    Cmd_05();// ����IM600Z����̬�����ݣ���С����λʱ����???��Ϊ��??0??
    Cmd_19();// ??������������??
    OLED_Init();
}
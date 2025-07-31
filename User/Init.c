//
// Created by ��� on 25-3-22.
//
#include "all.h"

void my_init() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
    //�������PWM��ʼ��
    HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4);
    //�����ض����ʼ��
    RetargetInit(&huart2);
    //UART�����жϳ�ʼ��
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // ����UART�����ж�
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DataBuff_UP, 200);   // ����UART�����ж�

    Set_Target_UartInit();
    //OLED��ʼ��
    OLED_Init();
    Set_Target_UartInit();

}
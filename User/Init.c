//
// Created by ��� on 25-3-22.
//
#include "all.h"
void my_init() {
    //��Դ��ѹADC������ʼ��
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    //PS2�ֱ���ʼ��
    PS2_SetInit();
    //��ʱ���жϳ�ʼ��
    HAL_TIM_Base_Start_IT(&htim6);
    //�������PWM��ʼ��
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    //�������PWM��ʼ��
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
    //�����ض����ʼ��
    RetargetInit(&huart2);
    //UART�����жϳ�ʼ��
    HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // ����UART�����ж�
//    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // ����UART�����ж�

    HAL_UART_Receive_DMA(&huart6,&rx_byte,1);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuffer_UP, 1);   // ����UART�����ж�
    //�����ʼ��
    motor_init();
    //PID��ʼ��
    PID_Init();

    //OLED��ʼ��
    OLED_Init();
    //�˲�����ʼ��
//    initialize_LowPassFilter(&filter_yaw, 0.85f);

}
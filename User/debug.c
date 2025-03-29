//
// Created by dyy on 2025/3/29.
//

#include "debug.h"

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  ���ݽ���
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������


}

void Set_Target_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//�ж��Ƿ��Ǵ���6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//��������жϱ�־����ֹ��һֱ���Ͻ����ж�

            Set_Target_UartIdleCallback(huart);//�����жϴ�����

        }
    }

}


void Set_Target_UartIdleCallback(UART_HandleTypeDef *huart)//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������
{
    HAL_UART_DMAStop(huart);//ֹͣ����DMA����

    //������յ������ݳ��ȣ����յ������ݳ��ȵ�����������洢���ȼ�ȥDMA���е�����������
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    memcpy(&stcAngle,&debugRvAll[2],8);

    yaw_last = yaw;
    yaw = (float) stcAngle.Angle[2] / 32768 * 180-yaw_offset;
    if (yaw - yaw_last > 180) {//����������
        yaw_total += (yaw - yaw_last) - 360;
    } else if (yaw - yaw_last < -180) {
        yaw_total += (yaw - yaw_last) + 360;
    } else {
        yaw_total += yaw - yaw_last;
    }
    data_length = 0;
    memset(&debugRvAll,0,data_length); //������ջ�����

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//ѭ���п������ڵ�DMA����

}


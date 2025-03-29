//
// Created by dyy on 2025/3/29.
//

#include "debug.h"

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  数据接收
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据


}

void Set_Target_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//判断是否是串口6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断

            Set_Target_UartIdleCallback(huart);//调用中断处理函数

        }
    }

}


void Set_Target_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    memcpy(&stcAngle,&debugRvAll[2],8);

    yaw_last = yaw;
    yaw = (float) stcAngle.Angle[2] / 32768 * 180-yaw_offset;
    if (yaw - yaw_last > 180) {//处理过零误差
        yaw_total += (yaw - yaw_last) - 360;
    } else if (yaw - yaw_last < -180) {
        yaw_total += (yaw - yaw_last) + 360;
    } else {
        yaw_total += yaw - yaw_last;
    }
    data_length = 0;
    memset(&debugRvAll,0,data_length); //清零接收缓冲区

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//循环中开启串口的DMA接收

}


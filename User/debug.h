//
// Created by Lenovo on 2025/3/29.
//

#ifndef MC_PROJ_DEBUG_H
#define MC_PROJ_DEBUG_H

#include "all.h"

void Set_Target_UartInit();
void DMA_UartIrqHandler(UART_HandleTypeDef *huart);
void DMA_UartIdleCallback(UART_HandleTypeDef *huart);//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������

#endif //MC_PROJ_DEBUG_H

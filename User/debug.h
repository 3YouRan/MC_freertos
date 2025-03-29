//
// Created by Lenovo on 2025/3/29.
//

#ifndef MC_PROJ_DEBUG_H
#define MC_PROJ_DEBUG_H

#include "all.h"

void Set_Target_UartInit();
void Set_Target_UartIrqHandler(UART_HandleTypeDef *huart);
void Set_Target_UartIdleCallback(UART_HandleTypeDef *huart);//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题

#endif //MC_PROJ_DEBUG_H

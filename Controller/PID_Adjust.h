//
// Created by ³Âè¤ on 2023-12-22.
//

#ifndef PID1_PID_ADJUST_H
#define PID1_PID_ADJUST_H
#include "main.h"
void USART_PID_Adjust(uint8_t Motor_n,uint8_t *Databuff);
float Get_Data(uint8_t* Data_Usart);
void USART_From_UP();
#endif //PID1_PID_ADJUST_H

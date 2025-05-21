//
// Created by Lenovo on 2025/5/21.
//

#include "debug.h"
#include"stdint.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];


//�ض��򴮿ڴ�ӡ�������ڲ���ϵͳ���ӡ������
//�ѽ��
void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);

    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);

    va_end(args);

    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)send_buf, length);
}



#include "debug.h"

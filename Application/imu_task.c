//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"

float yaw_last = 0;
float yaw = 0;
float yaw_total = 0;
void IMU_Task(void *argument){
    static uint8_t init_times = 0;
    static uint8_t init_flag = 0;
    while(1){
        U8 rxByte;
        while (Uart.UartFifo.Cnt > 0)
        {// 从fifo获取串口发来的数??

            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
            if (++Uart.UartFifo.Out >= FifoSize)
            {
                Uart.UartFifo.Out = 0;
            }
            __disable_irq();
            --Uart.UartFifo.Cnt;
            __enable_irq();

            Cmd_GetPkt(rxByte); //每收??1字节数据都填入该函数，当抓取到有效的数据包就会回调进?? Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
        }
        vTaskDelay(3);

    }
}
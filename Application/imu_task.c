//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"
void IM600_Task(void *argument){
    uint8_t rxByte;
    while(1){
        while (Uart.UartFifo.Cnt > 0)
        {// 从fifo获取串口发来的数据

            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
            if (++Uart.UartFifo.Out >= FifoSize)
            {
                Uart.UartFifo.Out = 0;
            }
            __disable_irq();
            --Uart.UartFifo.Cnt;
            __enable_irq();
            //每收到1字节数据都填入该函数，当抓取到有效的数据包就会回调进入Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
            Cmd_GetPkt(rxByte);
        }
    }
}
//
// Created by ��� on 25-3-22.
//

#include "all.h"
void IM600_Task(void *argument){
    uint8_t rxByte;
    while(1){
        while (Uart.UartFifo.Cnt > 0)
        {// ��fifo��ȡ���ڷ���������

            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
            if (++Uart.UartFifo.Out >= FifoSize)
            {
                Uart.UartFifo.Out = 0;
            }
            __disable_irq();
            --Uart.UartFifo.Cnt;
            __enable_irq();
            //ÿ�յ�1�ֽ����ݶ�����ú�������ץȡ����Ч�����ݰ��ͻ�ص�����Cmd_RxUnpack(U8 *buf, U8 DLen) ��������
            Cmd_GetPkt(rxByte);
        }
    }
}
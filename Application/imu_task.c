//
// Created by ��� on 25-3-22.
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
        {// ��fifo��ȡ���ڷ�������??

            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
            if (++Uart.UartFifo.Out >= FifoSize)
            {
                Uart.UartFifo.Out = 0;
            }
            __disable_irq();
            --Uart.UartFifo.Cnt;
            __enable_irq();

            Cmd_GetPkt(rxByte); //ÿ��??1�ֽ����ݶ�����ú�������ץȡ����Ч�����ݰ��ͻ�ص���?? Cmd_RxUnpack(U8 *buf, U8 DLen) ��������
        }
        vTaskDelay(3);

    }
}
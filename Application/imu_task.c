//
// Created by ��� on 25-3-22.
//

#include "all.h"


void IMU_Task(void *argument){
    uint8_t rxByte;

    while(1){
//        //IMU600
//        while (Uart.UartFifo.Cnt > 0)
//        {// ��fifo��ȡ���ڷ���������
//
//            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
//            if (++Uart.UartFifo.Out >= FifoSize)
//            {
//                Uart.UartFifo.Out = 0;
//            }
//            __disable_irq();
//            --Uart.UartFifo.Cnt;
//            __enable_irq();
//            //ÿ�յ�1�ֽ����ݶ�����ú�������ץȡ����Ч�����ݰ��ͻ�ص�����Cmd_RxUnpack(U8 *buf, U8 DLen) ��������
//            Cmd_GetPkt(rxByte);
//        }

          //MPU6050
//        MPU6050_Read_All(&hi2c2,&MPU6050);
//
////        printf("���ٶ� x:%.2f  y:%.2f  z:%.2f\r\n",MPU6050.Ax,MPU6050.Ay,MPU6050.Az);
////        printf("������ x:%.2f  y:%.2f  z:%.2f\n\r",MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);
////        printf("�¶� %.2f\n\r",MPU6050.Temperature);
//
//
//        yaw += (MPU6050.Gz+0.69f) * dt;
//        Kalman_getAngle(&KalmanZ,yaw,MPU6050.Gz,dt);
//        printf("yaw:%.2f\n\r",yaw);
//        vTaskDelay(20);

    }
}
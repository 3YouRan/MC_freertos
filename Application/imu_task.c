//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"


void IMU_Task(void *argument){
    uint8_t rxByte;

    while(1){
//        //IMU600
//        while (Uart.UartFifo.Cnt > 0)
//        {// 从fifo获取串口发来的数据
//
//            rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
//            if (++Uart.UartFifo.Out >= FifoSize)
//            {
//                Uart.UartFifo.Out = 0;
//            }
//            __disable_irq();
//            --Uart.UartFifo.Cnt;
//            __enable_irq();
//            //每收到1字节数据都填入该函数，当抓取到有效的数据包就会回调进入Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
//            Cmd_GetPkt(rxByte);
//        }

          //MPU6050
//        MPU6050_Read_All(&hi2c2,&MPU6050);
//
////        printf("加速度 x:%.2f  y:%.2f  z:%.2f\r\n",MPU6050.Ax,MPU6050.Ay,MPU6050.Az);
////        printf("陀螺仪 x:%.2f  y:%.2f  z:%.2f\n\r",MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);
////        printf("温度 %.2f\n\r",MPU6050.Temperature);
//
//
//        yaw += (MPU6050.Gz+0.69f) * dt;
//        Kalman_getAngle(&KalmanZ,yaw,MPU6050.Gz,dt);
//        printf("yaw:%.2f\n\r",yaw);
//        vTaskDelay(20);

    }
}
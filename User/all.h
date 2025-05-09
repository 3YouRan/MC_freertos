//
// Created by ��� on 25-3-22.
//

#ifndef MC_PROJ_ALL_H
#define MC_PROJ_ALL_H
//// ͷ�ļ�����
#include "main.h"
#include "base_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "retarget.h"
#include "PID_Adjust.h"
#include "PID_Control.h"
#include "driver_motor.h"
#include "Buzzer.h"
#include "im948_CMD.h"
#include "PS2.h"
#include "base_task.h"
#include "usart.h"
#include "bsp_usart.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "pid_task.h"
#include "uart_task.h"
#include "imu_task.h"
#include "PS2.h"
#include "oled.h"
#include "adc.h"
#include "oled_task.h"
#include "mpu6050.h"
#include "i2c.h"
#include "servo.h"
#include "pan_tile_task.h"
#include "HWT906.h"
#include <stdlib.h>
#include "filter.h"

////�궨��
#define RXSIZE 200
#define DATA_SIZE 14
#define DEBUG_RV_MXSIZE 255
//// ȫ�ֱ�������

extern motor motorA, motorB, motorC, motorD;
extern TaskHandle_t g_xUart6TaskHandle;

extern PID pid_speed_A;
extern PID pid_speed_B;
extern PID pid_speed_C;
extern PID pid_speed_D;

extern float Target_Speed_Inc;
extern float Target_Angle_Inc;
extern float Target_Angle_actual;

extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;
extern float Target_Speed_A_Now;
extern float Target_Speed_B_Now;
extern float Target_Speed_C_Now;
extern float Target_Speed_D_Now;
//����2���ջ�����
extern uint16_t RxLine;
extern uint8_t RxBuffer[1];
extern uint8_t DataBuff[200];
//����3���ջ�����
extern uint16_t RxLine_UP;
extern uint8_t RxBuffer_UP[1];
extern uint8_t DataBuff_UP[200];
//����������

//����������
extern U8 Data[9];
extern U16 MASK[16];
extern U16 Handkey;
extern struct_Ram_Uart Uart;

extern float Target_Angle;
extern PID pid_angle;
extern float angle_speed;
extern float angle_Car_total;
extern uint8_t sensor[4];
extern uint8_t Key1;

extern uint8_t direction ;
extern uint8_t mode_flag;//ģʽ1��ң�أ�ģʽ0��ѭ��

extern uint8_t L_TICK[2] ;//��ҡ������
extern uint8_t R_TICK[2] ;//��ҡ������

extern const unsigned char oled_asc2_1206[95][12];
extern const unsigned char oled_asc2_1608[95][16];
extern int16_t ADC_Value[1];
extern uint8_t Motor_Enable;

extern float yaw;
extern float yaw_last;
extern float yaw_total;

extern struct STime    stcTime;
extern struct SAcc     stcAcc;
extern struct SGyro    stcGyro;
extern struct SAngle   stcAngle;
extern struct SMag     stcMag;
extern struct SDStatus stcDStatus;
extern struct SPress 	stcPress;
extern struct SLonLat 	stcLonLat;
extern struct SGPSV    stcGPSV;
extern struct SQ       stcQ;
extern uint8_t debugRvAll[DEBUG_RV_MXSIZE];
extern LowPassFilter filter_yaw;

extern float servo1_angle;
extern float servo2_angle;
extern Base_status_t Base_target_status;
extern OdometryState_t Base_odometry;
extern uint8_t UART_num;
extern KalmanFilter kf;
////IRQ.c��Init.c�к���������
void my_init();
void Set_Target_UartInit();
void DMA_UartIrqHandler(UART_HandleTypeDef *huart);
void DMA_UartIdleCallback(UART_HandleTypeDef *huart);//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������


#endif //MC_PROJ_ALL_H

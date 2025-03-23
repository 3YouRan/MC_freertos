//
// Created by ��� on 25-3-22.
//

#ifndef MC_PROJ_ALL_H
#define MC_PROJ_ALL_H
// ͷ�ļ�����
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
#include "Kinematic_Analysis.h"
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

// ȫ�ֱ�������
extern QueueHandle_t g_xPS2QueueHandle; //PS2�ֱ����о��
extern float Target_Speed;

extern motor motorA, motorB, motorC, motorD;
extern TaskHandle_t g_xUart6TaskHandle;

extern PID pid_speed_A;
extern PID pid_position_A;
extern PID pid_speed_B;
extern PID pid_position_B;
extern PID pid_speed_C;
extern PID pid_position_C;
extern PID pid_speed_D;
extern PID pid_position_D;

extern float Target_Position;
extern float Target_Speed_Now;
extern float Target_Speed_Inc;

extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;
extern float Target_Speed_A_Now;
extern float Target_Speed_B_Now;
extern float Target_Speed_C_Now;
extern float Target_Speed_D_Now;
//����6���ջ�����
extern uint16_t RxLine;//指令长度
extern uint8_t RxBuffer[1];//串口接收缓冲
extern uint8_t DataBuff[200];//指令内容
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
extern float angle_speed;
extern float Vx;
extern float Vy;
extern uint8_t mode_flag;//ģʽ1��ң�أ�ģʽ0��ѭ��

extern uint8_t L_TICK[2] ;//��ҡ������
extern uint8_t R_TICK[2] ;//��ҡ������

extern const unsigned char oled_asc2_1206[95][12];
extern const unsigned char oled_asc2_1608[95][16];
extern int16_t ADC_Value[1];
extern uint8_t Motor_Enable;

extern float yaw;
extern float dt;
extern MPU6050_t MPU6050;
extern Kalman_t KalmanZ;

void my_init();



#endif //MC_PROJ_ALL_H

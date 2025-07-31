//
// Created by 陈瑜 on 25-3-22.
//

#ifndef MC_PROJ_ALL_H
#define MC_PROJ_ALL_H
//// 头文件包含
#include "adc.h"
#include <stdarg.h>
#include <stdio.h>
#include "stdlib.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "retarget.h"
#include "PID_Adjust.h"
#include "PID_Control.h"


#include "usart.h"

#include "string.h"
#include <stdio.h>
#include <stdlib.h>

#include "uart_task.h"

#include "oled.h"

#include "oled_task.h"

#include "i2c.h"
#include "servo.h"
#include "pan_tile_task.h"

#include <stdlib.h>
#include "filter.h"


////宏定义
#define RXSIZE 200
#define DATA_SIZE 14
#define DEBUG_RV_MXSIZE 255
//// 全局变量声明

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
//串口2接收缓冲区
extern uint16_t RxLine;
extern uint8_t RxBuffer[1];
extern uint8_t DataBuff[200];
//串口3接收缓冲区
extern uint16_t RxLine_UP;
extern uint8_t RxBuffer_UP[1];
extern uint8_t DataBuff_UP[200];
//陀螺仪数据



extern float Target_Angle;
extern PID pid_angle;
extern float angle_speed;
extern float angle_Car_total;
extern uint8_t sensor[4];
extern uint8_t Key1;

extern uint8_t direction ;
extern uint8_t mode_flag;//模式1：遥控，模式0：循迹

extern uint8_t L_TICK[2] ;//左摇杆数据
extern uint8_t R_TICK[2] ;//右摇杆数据

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

extern uint8_t UART_num;

extern uint8_t Angle_Enable;
extern float ax_imu,ay_imu;
extern uint8_t hit_flag;
extern uint8_t Buzzer_Flag ;
extern PID pid_pos;
extern uint8_t back_x_flag;
extern uint8_t back_y_flag;


extern int16_t Pulse_num1;
extern int16_t Pulse_num2;
extern uint8_t motor_rotate_flag;
extern float motor_angle1;
extern float motor_angle2;
extern uint8_t shoot_flag;
////IRQ.c，Init.c中函数的声明
void my_init();
void Set_Target_UartInit();
void DMA_UartIrqHandler(UART_HandleTypeDef *huart);
void DMA_Imu600_UartIdleCallback(UART_HandleTypeDef *huart);//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
void DMA_UP_UartIdleCallback(UART_HandleTypeDef *huart);//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
void usart_printf(const char* format, ...);
#endif //MC_PROJ_ALL_H

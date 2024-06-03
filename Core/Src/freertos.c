/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "event_groups.h"
#include "usart.h"
#include "retarget.h"
#include "driver_motor.h"
#include "PID_Control.h"
#include "PS2.h"
#include "queue.h"
#include "timers.h"
#include "im948_CMD.h"
#include "bsp_usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern motor motorA, motorB, motorC, motorD;
extern PID pid_speed_A;
extern float Target_Speed;
extern float Target_Position;
extern uint8_t Key1;

TaskHandle_t g_xUart6TaskHandle;//串口6打印任务句柄
TaskHandle_t g_xBaseControlTaskHandle;//底盘控制任务句柄
TaskHandle_t g_xPS2TaskHandle; //PS2任务句柄
TaskHandle_t g_xIM600TaskHandle; //陀螺仪任务句柄
TaskHandle_t g_xPIDTaskHandle; //陀螺仪任务句柄

QueueHandle_t g_xPS2QueueHandle; //PS2手柄队列句柄

//软件定时器句柄
TimerHandle_t g_xTimerPIDHandle;


EventGroupHandle_t g_xEventGroupPIDHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void UART6_Task(void *argument);
void IM600_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  motor_init();
  PID_Init();
    Cmd_03();//唤醒传感??
    Cmd_12(5, 255, 0,  0, 3, 100, 2, 4, 9, 0x40);//设置IM600设备参数
    Cmd_05();// 归零IM600Z轴姿态角数据，以小车复位时的姿???角为角??0??
    Cmd_19();// ??启数据主动上??
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
//  g_xTimerPIDHandle=xTimerCreate("PID_Timer",10,pdTRUE,NULL,PID_Timer_Callback);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  g_xPS2QueueHandle= xQueueCreate(10,sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(UART6_Task,"UART6_Task",128,NULL,osPriorityNormal,&g_xUart6TaskHandle);
  xTaskCreate(Base_Control,"Base_Control",128,NULL,osPriorityNormal,&g_xBaseControlTaskHandle);
  xTaskCreate(PS2_Task,"PS2_Task",128,NULL,osPriorityNormal,&g_xPS2TaskHandle);
  xTaskCreate(IM600_Task,"IM600_Task",128,NULL,osPriorityNormal,&g_xIM600TaskHandle);
  xTaskCreate(PID_Task,"PID_Task",128,NULL,osPriorityNormal+1,&g_xPIDTaskHandle);
  //  xTimerStart(g_xTimerPIDHandle,0);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
extern float Target_Speed_Now;
extern float Target_Angle;
extern float angle_Car;
extern PID pid_angle;
extern float angle_speed;
extern float angle_Car_total;
extern uint8_t sensor[4];
void UART6_Task(void *argument){
    while(1){
//        printf("UART2:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n\r",pid_angle.kp,pid_angle.output,Target_Angle,motorA.speed,angle_Car_total,angle_speed,Key1);
        vTaskDelay(55);
        printf("UART2:%d,%d,%d,%d\r\n",sensor[0],sensor[1],sensor[2],sensor[3]);
    }
}
extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;
extern struct_Ram_Uart Uart;
float angle_total=0;
extern float angle_Car;

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
/* USER CODE END Application */


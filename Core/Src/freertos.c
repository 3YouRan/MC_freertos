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
  RetargetInit(&huart6);
  motor_init();
  PID_Init();

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
void UART6_Task(void *argument){
    while(1){
        printf("UART6:%.2f,%.2f,%.2f,%.2f,%.2f,%d\n\r",pid_speed_A.kp,pid_speed_A.kd,Target_Speed_Now,motorA.speed,Target_Speed,Key1);
        vTaskDelay(55);
    }
}

/* USER CODE END Application */


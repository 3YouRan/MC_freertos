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
#include "all.h"
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

extern PID pid_speed_A;
extern float Target_Speed;
extern float Target_Position;
extern uint8_t Key1;

TaskHandle_t g_xUart6TaskHandle;//����6��ӡ������
TaskHandle_t g_xBaseControlTaskHandle;//���̿���������
TaskHandle_t g_xPS2TaskHandle; //PS2������
TaskHandle_t g_xIM600TaskHandle; //������������
TaskHandle_t g_xPIDTaskHandle; //������������
TaskHandle_t g_xOLEDTaskHandle; //������������
TaskHandle_t g_xPAN_tileTaskHandle; //������������
TaskHandle_t g_xOdemetryTaskHandle; //������������
TaskHandle_t g_xBUZZERTaskHandle; //������������
TaskHandle_t g_xBackTaskHandle; //������������

//�����ʱ�����
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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //底层任务
  xTaskCreate(UART6_Task,"UART6_Task",128,NULL,osPriorityNormal-1,&g_xUart6TaskHandle);
  xTaskCreate(OLED_Task,"OLED_Task",256,NULL,osPriorityNormal-1,&g_xOLEDTaskHandle);
  xTaskCreate(pan_tile_task,"PAN_tile_Task",1024,NULL,osPriorityNormal+5,&g_xPAN_tileTaskHandle);
//    xTaskCreate(Back_Task,"Back_Task",256,NULL,osPriorityNormal+4,&g_xBackTaskHandle);
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




/* USER CODE END Application */


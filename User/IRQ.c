//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"

uint16_t RxLine = 0;//指令长度
uint8_t RxBuffer[1];//串口接收缓冲
uint8_t DataBuff[200];//指令内容
uint8_t UART_num = 0;
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // UART_num=2;
        RxLine++;                            // 接收行数加1
        DataBuff[RxLine-1]=RxBuffer[0];       // 将接收到的数据存入缓冲区
        if(RxBuffer[0]=='!')                  // 判断是否接收到感叹号
        {
            printf("RXLen=%d\r\n",RxLine);
            // for(int i=0;i<RxLine;i++)
            //     printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1,DataBuff);             // 调整USART PID


            memset(DataBuff,0,sizeof(DataBuff));   // 清空接收缓冲区
            RxLine=0;                           // 接收行数清零
        }
        RxBuffer[0]=0;                        // 重置接收缓冲区

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 重新启动UART接收中断
    }

}

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  数据接收
    // __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    // HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataBuff_UP, 200);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{

    if(huart->Instance == huart3.Instance)//判断是否是串口3
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断

            DMA_UP_UartIdleCallback(huart);//调用中断处理函数

        }
    }
}

void DMA_Imu600_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //将接收到的数据存入数组
//    printf("UART6:%x,%x,%x,%x,%x\r\n",debugRvAll[0],debugRvAll[1],debugRvAll[2],debugRvAll[3],debugRvAll[4]);
    // imu600_parse(&debugRvAll[3]);
    memset(&debugRvAll,0,data_length); //清零接收缓冲区

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//循环中开启串口的DMA接收

}
void DMA_UP_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //将接收到的数据存入数组
    // printf("UART3");
    USART_PID_Adjust(1,DataBuff_UP);
    memset(&DataBuff_UP,0,data_length); //清零接收缓冲区
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DataBuff_UP, 200);   // 启动UART接收中断

}


int times1=0;
short encoder_now1=0;
short encoder_now2=0;
short encoder_now3=0;
short encoder_now4=0;

float Target_Speed_Inc=3;
float Target_Angle_Inc=1.5f;

/* USER CODE END 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */

    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t cnt1 = 0;
    static uint16_t cnt2 = 0;
    if (htim->Instance == TIM12) {
        cnt1++;
        if(cnt1 == Pulse_num1||Pulse_num1==0) {
            cnt1 = 0;
            Pulse_num1=0;
            motor_angle1=0;

            HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???

            HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???

            HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???


            HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);//????PWM???

            // motor_angle1=0;
            // HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
        }
    }

    if (htim->Instance == TIM3) {
        cnt2++;
        if(cnt2 == Pulse_num2||Pulse_num2==0) {
            cnt2 = 0;
            Pulse_num2=0;
            motor_angle2=0;


            HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);

            HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);

            HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
            // motor_angle1=0;
            // HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
        }
    }
}






#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];

void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);

    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);

    va_end(args);

    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)send_buf, length);
}
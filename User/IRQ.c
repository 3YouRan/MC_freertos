//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {
        // 处理接收到的数据

        if (FifoSize > Uart.UartFifo.Cnt)
        {
            Uart.UartFifo.RxBuf[Uart.UartFifo.In] = rx_byte;
            if(++Uart.UartFifo.In >= FifoSize)
            {
                Uart.UartFifo.In = 0;
            }
            ++Uart.UartFifo.Cnt;
        }

        // 重新启用接收中断，以便继续接收数??
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
    if (huart->Instance == USART2) {

        RxLine++;                            // 接收行数加1
        DataBuff[RxLine-1]=RxBuffer[0];       // 将接收到的数据存入缓冲区
        if(RxBuffer[0]=='!')                  // 判断是否接收到感叹号
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1);             // 调整USART PID

            memset(DataBuff,0,sizeof(DataBuff));   // 清空接收缓冲区
            RxLine=0;                           // 接收行数清零
        }
        RxBuffer[0]=0;                        // 重置接收缓冲区

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 重新启动UART接收中断
    }
}
int times1=0;
short encoder_now1=0;
short encoder_now2=0;
short encoder_now3=0;
short encoder_now4=0;
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

    if (htim==&htim6){
        times1++;
        encoder_now1=(short )(__HAL_TIM_GET_COUNTER(ENCODER1));
        encoder_now2=(short )(__HAL_TIM_GET_COUNTER(ENCODER2));
        encoder_now3=(short )(__HAL_TIM_GET_COUNTER(ENCODER3));
        encoder_now4=(short )(__HAL_TIM_GET_COUNTER(ENCODER4));

        motorA.totalCount+=encoder_now1;//绱姞鑴夊啿鏁?
        motorB.totalCount+=encoder_now2;
        motorC.totalCount+=encoder_now3;
        motorD.totalCount+=encoder_now4;
        __HAL_TIM_SET_COUNTER(ENCODER1,0);
        __HAL_TIM_SET_COUNTER(ENCODER2,0);
        __HAL_TIM_SET_COUNTER(ENCODER3,0);
        __HAL_TIM_SET_COUNTER(ENCODER4,0);
        motorA.speed=(float )encoder_now1/(4*30*500)*1000*60;
        motorB.speed=(float )encoder_now2/(4*30*500)*1000*60; //rpm
        motorC.speed=(float )encoder_now3/(4*30*500)*1000*60;
        motorD.speed=(float )encoder_now4/(4*30*500)*1000*60;
        if(times1==10){
            times1=0;
            //加速
            if (Target_Speed_A_Now<Target_Speed_A){
                Target_Speed_A_Now+=Target_Speed_Inc;
            }else if(Target_Speed_A_Now>Target_Speed_A){
                Target_Speed_A_Now-=Target_Speed_Inc;
            }
            if (Target_Speed_B_Now<Target_Speed_B){
                Target_Speed_B_Now+=Target_Speed_Inc;
            }else if(Target_Speed_B_Now>Target_Speed_B){
                Target_Speed_B_Now-=Target_Speed_Inc;
            }
            if (Target_Speed_C_Now<Target_Speed_C){
                Target_Speed_C_Now+=Target_Speed_Inc;
            }else if(Target_Speed_C_Now>Target_Speed_C){
                Target_Speed_C_Now-=Target_Speed_Inc;
            }
            if (Target_Speed_D_Now<Target_Speed_D){
                Target_Speed_D_Now+=Target_Speed_Inc;
            }else if(Target_Speed_D_Now>Target_Speed_D){
                Target_Speed_D_Now-=Target_Speed_Inc;
            }


        }
    }
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}
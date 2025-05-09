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
        UART_num=2;
        RxLine++;                            // 接收行数加1
        DataBuff[RxLine-1]=RxBuffer[0];       // 将接收到的数据存入缓冲区
        if(RxBuffer[0]=='!')                  // 判断是否接收到感叹号
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1,DataBuff);             // 调整USART PID

            memset(DataBuff,0,sizeof(DataBuff));   // 清空接收缓冲区
            RxLine=0;                           // 接收行数清零
        }
        RxBuffer[0]=0;                        // 重置接收缓冲区

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 重新启动UART接收中断
    }
    if (huart->Instance == USART3) {
        UART_num=3;
        RxLine_UP++;                            // 接收行数加1
        DataBuff_UP[RxLine_UP-1]=RxBuffer_UP[0];       // 将接收到的数据存入缓冲区
        if(RxBuffer_UP[0]=='!')                  // 判断是否接收到感叹号
        {
            printf("RXLen_UP=%d\r\n",RxLine_UP);
            for(int i=0;i<RxLine_UP;i++)
                printf("UART DataBuff_UP[%d] = %c\r\n",i,DataBuff_UP[i]);
            USART_PID_Adjust(1,DataBuff_UP);             // 调整USART PID


            memset(DataBuff_UP,0,sizeof(DataBuff_UP));   // 清空接收缓冲区
            RxLine_UP=0;                           // 接收行数清零
        }
        RxBuffer_UP[0]=0;                        // 重置接收缓冲区

        HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuffer_UP, 1);   // 重新启动UART接收中断
    }
}

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  数据接收
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//判断是否是串口6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断

            DMA_UartIdleCallback(huart);//调用中断处理函数

        }
    }

}


void DMA_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    switch(debugRvAll[1]){
        case 0x53:
            memcpy(&stcAngle,&debugRvAll[2],8);
            break;
        case 0x51:
            memcpy(&stcAcc,&debugRvAll[2],8);

            break;
    }
//    memcpy(&stcAngle,&debugRvAll[2],8);
//    printf("IRQ: %x\r\n",debugRvAll[1]);
    yaw_last = yaw;
    yaw = (float) stcAngle.Angle[2] / 32768 * 180;
    if (yaw - yaw_last > 180) {//处理过零误差
        yaw_total += (yaw - yaw_last) - 360;
    } else if (yaw - yaw_last < -180) {
        yaw_total += (yaw - yaw_last) + 360;
    } else {
        yaw_total += yaw - yaw_last;
    }
    data_length = 0;
    memset(&debugRvAll,0,data_length); //清零接收缓冲区

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//循环中开启串口的DMA接收

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
    //calculate_odometry(motorA.speed,motorB.speed,motorC.speed,motorD.speed,&Base_odometry,0.002f);
    if (htim==&htim6){// 2ms
        times1++;
        encoder_now1=(short )(__HAL_TIM_GET_COUNTER(ENCODER1));
        encoder_now2=(short )(__HAL_TIM_GET_COUNTER(ENCODER2));
        encoder_now3=(short )(__HAL_TIM_GET_COUNTER(ENCODER3));
        encoder_now4=(short )(__HAL_TIM_GET_COUNTER(ENCODER4));
        motorA.totalCount+=encoder_now1;
        motorB.totalCount+=encoder_now2;
        motorC.totalCount+=encoder_now3;
        motorD.totalCount+=encoder_now4;
        __HAL_TIM_SET_COUNTER(ENCODER1,0);
        __HAL_TIM_SET_COUNTER(ENCODER2,0);
        __HAL_TIM_SET_COUNTER(ENCODER3,0);
        __HAL_TIM_SET_COUNTER(ENCODER4,0);
        motorA.speed=(float )encoder_now1/(4*28*500)*500*60;
        motorB.speed=-(float )encoder_now2/(4*28*500)*500*60; //rpm
        motorC.speed=(float )encoder_now3/(4*28*500)*500*60;
        motorD.speed=-(float )encoder_now4/(4*30*500)*500*60;
        if(times1==5){//10ms
            // 转换IMU加速度到全局坐标系
            float a_global[2];
            imu_to_global(stcAcc.a[0]/32768*16*G, stcAcc.a[1]/32768*16*G, yaw_total, &a_global[0], &a_global[1]);

            // 预测步骤
            kalman_predict(&kf, a_global);

            // 更新步骤
            float z[] = {Base_odometry.x, Base_odometry.y,};
            kalman_update(&kf, z);

            // 输出结果
//            printf("Estimated Position: (%.2f, %.2f)\n",
//                   kf.x[0], kf.x[1]);
            calculate_odometry(motorA.speed,motorB.speed,motorC.speed,motorD.speed,&Base_odometry,0.01f);
        }
        if(times1==10){//20ms
            times1=0;
            //目标速度爬坡
            if (motorA.TargetSpeed_now<motorA.TargetSpeed){
                motorA.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorA.TargetSpeed_now>motorA.TargetSpeed){
                motorA.TargetSpeed_now-=Target_Speed_Inc;
            }else if(fabsf(motorA.TargetSpeed_now-motorA.TargetSpeed)<Target_Speed_Inc){
                motorA.TargetSpeed_now=motorA.TargetSpeed;
            }
            if (motorB.TargetSpeed_now<motorB.TargetSpeed){
                motorB.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorB.TargetSpeed_now>motorB.TargetSpeed){
                motorB.TargetSpeed_now-=Target_Speed_Inc;
            } else if(fabsf(motorB.TargetSpeed_now-motorB.TargetSpeed)<Target_Speed_Inc){
                motorB.TargetSpeed_now=motorB.TargetSpeed;
            }
            if (motorC.TargetSpeed_now<motorC.TargetSpeed){
                motorC.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorC.TargetSpeed_now>motorC.TargetSpeed){
                motorC.TargetSpeed_now-=Target_Speed_Inc;
            }else if(fabsf(motorC.TargetSpeed_now-motorC.TargetSpeed)<Target_Speed_Inc){
                motorC.TargetSpeed_now=motorC.TargetSpeed;
            }
            if (motorD.TargetSpeed_now<motorD.TargetSpeed){
                motorD.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorD.TargetSpeed_now>motorD.TargetSpeed){
                motorD.TargetSpeed_now-=Target_Speed_Inc;
            } else if(fabsf(motorD.TargetSpeed_now-motorD.TargetSpeed)<Target_Speed_Inc){
                motorD.TargetSpeed_now=motorD.TargetSpeed;
            }
            //目标角度爬坡
            if(Target_Angle_actual-Target_Angle>Target_Angle_Inc){
                Target_Angle_actual-=Target_Angle_Inc;
            }else if(Target_Angle_actual-Target_Angle<-Target_Angle_Inc){
                Target_Angle_actual+=Target_Angle_Inc;
            }else if(fabsf(Target_Angle_actual-Target_Angle)<Target_Angle_Inc){
                Target_Angle_actual=Target_Angle;
            }
        }
    }
//    if (htim->Instance == TIM12) {//1ms
//
//
//    }
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }

    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}
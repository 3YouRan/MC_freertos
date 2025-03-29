//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"
uint16_t imu_init_times=0;
uint8_t imu_init_flag=1;
float yaw_offset=0;
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {

        CopeSerial2Data(rx_byte);
//        if(imu_init_flag==1){//等待imu角度稳定
//            imu_init_times++;
//            if(imu_init_times==20000){
//                imu_init_flag=0;
//                yaw_offset=(float) stcAngle.Angle[2] / 32768 * 180;
//            }
//        }else if(imu_init_flag==0) {
//            yaw_last = yaw;
//            yaw = (float) stcAngle.Angle[2] / 32768 * 180-yaw_offset;
//            if (yaw - yaw_last > 180) {//处理过零误差
//                yaw_total += (yaw - yaw_last) - 360;
//            } else if (yaw - yaw_last < -180) {
//                yaw_total += (yaw - yaw_last) + 360;
//            } else {
//                yaw_total += yaw - yaw_last;
//            }
//        }
        yaw_last = yaw;
        yaw = (float) stcAngle.Angle[2] / 32768 * 180-yaw_offset;
        if (yaw - yaw_last > 180) {//处理过零误差
            yaw_total += (yaw - yaw_last) - 360;
        } else if (yaw - yaw_last < -180) {
            yaw_total += (yaw - yaw_last) + 360;
        } else {
            yaw_total += yaw - yaw_last;
        }
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // 启动UART接收中断

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
//float yaw = 0;
//float dt = 0.02f;
//MPU6050_t MPU6050;
//Kalman_t KalmanZ = {
//        .Q_angle = 0.008f,
//        .Q_bias = 0.003f,
//        .R_measure = 0.1f,
//};
//float Gyro_Z_Offset = 0;
//uint8_t IMU_times = 0;
//uint8_t yaw_flag = 0;

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

    if (htim==&htim6){// 2ms
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

            if(Target_Angle_actual-Target_Angle>Target_Speed_Inc){
                Target_Angle_actual-=Target_Angle_Inc;
            }else if(Target_Angle_actual-Target_Angle<-Target_Speed_Inc){
                Target_Angle_actual+=Target_Angle_Inc;
            }else{
                Target_Angle_actual=Target_Angle;
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
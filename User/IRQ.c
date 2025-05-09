//
// Created by ��� on 25-3-22.
//

#include "all.h"

uint16_t RxLine = 0;//ָ���
uint8_t RxBuffer[1];//���ڽ��ջ���
uint8_t DataBuff[200];//ָ������
uint8_t UART_num = 0;
// UART������ɻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2) {
        UART_num=2;
        RxLine++;                            // ����������1
        DataBuff[RxLine-1]=RxBuffer[0];       // �����յ������ݴ��뻺����
        if(RxBuffer[0]=='!')                  // �ж��Ƿ���յ���̾��
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1,DataBuff);             // ����USART PID

            memset(DataBuff,0,sizeof(DataBuff));   // ��ս��ջ�����
            RxLine=0;                           // ������������
        }
        RxBuffer[0]=0;                        // ���ý��ջ�����

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // ��������UART�����ж�
    }
    if (huart->Instance == USART3) {
        UART_num=3;
        RxLine_UP++;                            // ����������1
        DataBuff_UP[RxLine_UP-1]=RxBuffer_UP[0];       // �����յ������ݴ��뻺����
        if(RxBuffer_UP[0]=='!')                  // �ж��Ƿ���յ���̾��
        {
            printf("RXLen_UP=%d\r\n",RxLine_UP);
            for(int i=0;i<RxLine_UP;i++)
                printf("UART DataBuff_UP[%d] = %c\r\n",i,DataBuff_UP[i]);
            USART_PID_Adjust(1,DataBuff_UP);             // ����USART PID


            memset(DataBuff_UP,0,sizeof(DataBuff_UP));   // ��ս��ջ�����
            RxLine_UP=0;                           // ������������
        }
        RxBuffer_UP[0]=0;                        // ���ý��ջ�����

        HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuffer_UP, 1);   // ��������UART�����ж�
    }
}

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  ���ݽ���
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//�ж��Ƿ��Ǵ���6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//��������жϱ�־����ֹ��һֱ���Ͻ����ж�

            DMA_UartIdleCallback(huart);//�����жϴ�����

        }
    }

}


void DMA_UartIdleCallback(UART_HandleTypeDef *huart)//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������
{
    HAL_UART_DMAStop(huart);//ֹͣ����DMA����

    //������յ������ݳ��ȣ����յ������ݳ��ȵ�����������洢���ȼ�ȥDMA���е�����������
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
    if (yaw - yaw_last > 180) {//����������
        yaw_total += (yaw - yaw_last) - 360;
    } else if (yaw - yaw_last < -180) {
        yaw_total += (yaw - yaw_last) + 360;
    } else {
        yaw_total += yaw - yaw_last;
    }
    data_length = 0;
    memset(&debugRvAll,0,data_length); //������ջ�����

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//ѭ���п������ڵ�DMA����

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
            // ת��IMU���ٶȵ�ȫ������ϵ
            float a_global[2];
            imu_to_global(stcAcc.a[0]/32768*16*G, stcAcc.a[1]/32768*16*G, yaw_total, &a_global[0], &a_global[1]);

            // Ԥ�ⲽ��
            kalman_predict(&kf, a_global);

            // ���²���
            float z[] = {Base_odometry.x, Base_odometry.y,};
            kalman_update(&kf, z);

            // ������
//            printf("Estimated Position: (%.2f, %.2f)\n",
//                   kf.x[0], kf.x[1]);
            calculate_odometry(motorA.speed,motorB.speed,motorC.speed,motorD.speed,&Base_odometry,0.01f);
        }
        if(times1==10){//20ms
            times1=0;
            //Ŀ���ٶ�����
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
            //Ŀ��Ƕ�����
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
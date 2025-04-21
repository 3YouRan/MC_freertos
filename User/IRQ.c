//
// Created by ��� on 25-3-22.
//

#include "all.h"

uint16_t RxLine = 0;//ָ���
uint8_t RxBuffer[1];//���ڽ��ջ���
uint8_t DataBuff[200];//ָ������

// UART������ɻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2) {

        RxLine++;                            // ����������1
        DataBuff[RxLine-1]=RxBuffer[0];       // �����յ������ݴ��뻺����
        if(RxBuffer[0]=='!')                  // �ж��Ƿ���յ���̾��
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1);             // ����USART PID

            memset(DataBuff,0,sizeof(DataBuff));   // ��ս��ջ�����
            RxLine=0;                           // ������������
        }
        RxBuffer[0]=0;                        // ���ý��ջ�����

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // ��������UART�����ж�
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

    memcpy(&stcAngle,&debugRvAll[2],8);

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
        motorA.speed=(float )encoder_now1/(4*30*500)*1000*60;
        motorB.speed=(float )encoder_now2/(4*30*500)*1000*60; //rpm
        motorC.speed=(float )encoder_now3/(4*30*500)*1000*60;
        motorD.speed=(float )encoder_now4/(4*30*500)*1000*60;
        if(times1==10){
            times1=0;
            //Ŀ���ٶ�����
            if (Target_Speed_A_Now<Target_Speed_A){
                Target_Speed_A_Now+=Target_Speed_Inc;
            }else if(Target_Speed_A_Now>Target_Speed_A){
                Target_Speed_A_Now-=Target_Speed_Inc;
            }else if(fabsf(Target_Speed_A_Now-Target_Speed_A)<Target_Speed_Inc){
                Target_Speed_A_Now=Target_Speed_A;
            }
            if (Target_Speed_B_Now<Target_Speed_B){
                Target_Speed_B_Now+=Target_Speed_Inc;
            }else if(Target_Speed_B_Now>Target_Speed_B){
                Target_Speed_B_Now-=Target_Speed_Inc;
            } else if(fabsf(Target_Speed_B_Now-Target_Speed_B)<Target_Speed_Inc){
                Target_Speed_B_Now=Target_Speed_B;
            }
            if (Target_Speed_C_Now<Target_Speed_C){
                Target_Speed_C_Now+=Target_Speed_Inc;
            }else if(Target_Speed_C_Now>Target_Speed_C){
                Target_Speed_C_Now-=Target_Speed_Inc;
            }else if(fabsf(Target_Speed_C_Now-Target_Speed_C)<Target_Speed_Inc){
                Target_Speed_C_Now=Target_Speed_C;
            }
            if (Target_Speed_D_Now<Target_Speed_D){
                Target_Speed_D_Now+=Target_Speed_Inc;
            }else if(Target_Speed_D_Now>Target_Speed_D){
                Target_Speed_D_Now-=Target_Speed_Inc;
            } else if(fabsf(Target_Speed_D_Now-Target_Speed_D)<Target_Speed_Inc){
                Target_Speed_D_Now=Target_Speed_D;
            }
            //Ŀ��Ƕ�����
            if(Target_Angle_actual-Target_Angle>Target_Speed_Inc){
                Target_Angle_actual-=Target_Angle_Inc;
            }else if(Target_Angle_actual-Target_Angle<-Target_Speed_Inc){
                Target_Angle_actual+=Target_Angle_Inc;
            }else if(fabsf(Target_Angle_actual-Target_Angle)<Target_Angle_Inc){
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
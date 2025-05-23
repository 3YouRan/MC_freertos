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

}

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  ���ݽ���
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataBuff_UP, 200);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//�ж��Ƿ��Ǵ���6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//��������жϱ�־����ֹ��һֱ���Ͻ����ж�

            DMA_Imu600_UartIdleCallback(huart);//�����жϴ�����

        }
    }
    if(huart->Instance == huart3.Instance)//�ж��Ƿ��Ǵ���2
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//��������жϱ�־����ֹ��һֱ���Ͻ����ж�

            DMA_UP_UartIdleCallback(huart);//�����жϴ�����

        }
    }
}
void imu600_parse(uint8_t *buf){
    U16 ctl;
    U8 L;
    U8 tmpU8;
    U16 tmpU16;
    U32 tmpU32;
    F32 tmpX, tmpY, tmpZ, tmpAbs;
    if (buf[0]== 0x11) // ��ȡ���ĵĹ������� �ظ��������ϱ�
    {
            ctl = ((U16)buf[2] << 8) | buf[1];// �ֽ�[2-1] Ϊ���ܶ��ı�ʶ��ָʾ��ǰ��������Щ����
    //            Dbp("\t subscribe tag: 0x%04X\r\n", ctl);
    //            Dbp("\t ms: %u\r\n", (U32)(((U32)buf[6]<<24) | ((U32)buf[5]<<16) | ((U32)buf[4]<<8) | ((U32)buf[3]<<0))); // �ֽ�[6-3] Ϊģ�鿪�����ʱ���(��λms)

        L =7; // �ӵ�7�ֽڿ�ʼ���� ���ı�ʶtag������ʣ�µ�����
        if ((ctl & 0x0001) != 0)
        {// ���ٶ�xyz ȥ�������� ʹ��ʱ��*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2;// Dbp("\taX: %.3f\r\n", tmpX); // x���ٶ�aX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2;// Dbp("\taY: %.3f\r\n", tmpY); // y���ٶ�aY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\taZ: %.3f\r\n", tmpZ); // z���ٶ�aZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\ta_abs: %.3f\r\n", tmpAbs); // 3��ϳɵľ���ֵ
            ax_imu=tmpX;
            ay_imu=tmpY;
        }
        if ((ctl & 0x0002) != 0)
        {// ���ٶ�xyz ���������� ʹ��ʱ��*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAX: %.3f\r\n", tmpX); // x���ٶ�AX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAY: %.3f\r\n", tmpY); // y���ٶ�AY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAZ: %.3f\r\n", tmpZ); // z���ٶ�AZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tA_abs: %.3f\r\n", tmpAbs); // 3��ϳɵľ���ֵ
        }
        if ((ctl & 0x0004) != 0)
        {// ���ٶ�xyz ʹ��ʱ��*scaleAngleSpeed ��/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGX: %.3f\r\n", tmpX); // x���ٶ�GX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGY: %.3f\r\n", tmpY); // y���ٶ�GY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGZ: %.3f\r\n", tmpZ); // z���ٶ�GZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tG_abs: %.3f\r\n", tmpAbs); // 3��ϳɵľ���ֵ
        }
        if ((ctl & 0x0008) != 0)
        {// �ų�xyz ʹ��ʱ��*scaleMag uT
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCX: %.3f\r\n", tmpX); // x�ų�CX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCY: %.3f\r\n", tmpY); // y�ų�CY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCZ: %.3f\r\n", tmpZ); // z�ų�CZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tC_abs: %.3f\r\n", tmpAbs); // 3��ϳɵľ���ֵ
        }
        if ((ctl & 0x0010) != 0)
        {// �¶� ��ѹ �߶�
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; Dbp("\ttemperature: %.2f\r\n", tmpX); // �¶�

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// ��24λ�������λΪ1�����ֵΪ��������תΪ32λ������ֱ�Ӳ���ff����
            tmpY = (S32)tmpU32 * scaleAirPressure; L += 3; Dbp("\tairPressure: %.3f\r\n", tmpY); // ��ѹ

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// ��24λ�������λΪ1�����ֵΪ��������תΪ32λ������ֱ�Ӳ���ff����
            tmpZ = (S32)tmpU32 * scaleHeight; L += 3; Dbp("\theight: %.3f\r\n", tmpZ); // �߶�
        }
        if ((ctl & 0x0020) != 0)
        {// ��Ԫ�� wxyz ʹ��ʱ��*scaleQuat
            tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tw: %.3f\r\n", tmpAbs); // w
            tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tx: %.3f\r\n", tmpX); // x
            tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\ty: %.3f\r\n", tmpY); // y
            tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tz: %.3f\r\n", tmpZ); // z
        }
        if ((ctl & 0x0040) != 0)
        {// ŷ����xyz ʹ��ʱ��*scaleAngle
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleX: %.3f\r\n", tmpX); // x�Ƕ�
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleY: %.3f\r\n", tmpY); // y�Ƕ�
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleZ: %.3f\r\n", tmpZ); // z�Ƕ�
            yaw_last = yaw;
            yaw=tmpZ;
            if(yaw-yaw_last>180){
                yaw_total+=yaw-yaw_last-360;
            }else if(yaw-yaw_last<-180){
                yaw_total+=yaw-yaw_last+360;
            }else{
                yaw_total+=yaw-yaw_last;
            }
//            usart_printf("%.2f,%.2f,%.2f!",tmpX,tmpY,tmpZ);
        }
        if ((ctl & 0x0080) != 0)
        {// xyz �ռ�λ�� ��λmm תΪ m
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetX: %.3f\r\n", tmpX); // x����
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetY: %.3f\r\n", tmpY); // y����
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetZ: %.3f\r\n", tmpZ); // z����
        }
        if ((ctl & 0x0100) != 0)
        {// ��������
            tmpU32 = (U32)(((U32)buf[L+3]<<24) | ((U32)buf[L+2]<<16) | ((U32)buf[L+1]<<8) | ((U32)buf[L]<<0)); L += 4; Dbp("\tsteps: %u\r\n", tmpU32); // �Ʋ���
            tmpU8 = buf[L]; L += 1;
            Dbp("\t walking: %s\r\n", (tmpU8 & 0x01)?  "yes" : "no"); // �Ƿ�����·
            Dbp("\t running: %s\r\n", (tmpU8 & 0x02)?  "yes" : "no"); // �Ƿ����ܲ�
            Dbp("\t biking: %s\r\n",  (tmpU8 & 0x04)?  "yes" : "no"); // �Ƿ����ﳵ
            Dbp("\t driving: %s\r\n", (tmpU8 & 0x08)?  "yes" : "no"); // �Ƿ��ڿ���
        }
        if ((ctl & 0x0200) != 0)
        {// ���ٶ�xyz ȥ�������� ʹ��ʱ��*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasX: %.3f\r\n", tmpX); // x���ٶ�asX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasY: %.3f\r\n", tmpY); // y���ٶ�asY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasZ: %.3f\r\n", tmpZ); // z���ٶ�asZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\tas_abs: %.3f\r\n", tmpAbs); // 3��ϳɵľ���ֵ


        }
        if ((ctl & 0x0400) != 0)
        {// ADC��ֵ
            tmpU16 = (U16)(((U16)buf[L+1]<<8) | ((U16)buf[L]<<0)); L += 2; Dbp("\tadc: %u\r\n", tmpU16); // ��λmv
        }
        if ((ctl & 0x0800) != 0)
        {// GPIO1��ֵ
            tmpU8 = buf[L]; L += 1;
            Dbp("\t GPIO1  M:%X, N:%X\r\n", (tmpU8>>4)&0x0f, (tmpU8)&0x0f);
        }
    }
}

void DMA_Imu600_UartIdleCallback(UART_HandleTypeDef *huart)//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������
{
    HAL_UART_DMAStop(huart);//ֹͣ����DMA����

    //������յ������ݳ��ȣ����յ������ݳ��ȵ�����������洢���ȼ�ȥDMA���е�����������
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //�����յ������ݴ�������
//    printf("UART6:%x,%x,%x,%x,%x\r\n",debugRvAll[0],debugRvAll[1],debugRvAll[2],debugRvAll[3],debugRvAll[4]);
    imu600_parse(&debugRvAll[3]);
    memset(&debugRvAll,0,data_length); //������ջ�����

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//ѭ���п������ڵ�DMA����

}
void DMA_UP_UartIdleCallback(UART_HandleTypeDef *huart)//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������
{
    HAL_UART_DMAStop(huart);//ֹͣ����DMA����

    //������յ������ݳ��ȣ����յ������ݳ��ȵ�����������洢���ȼ�ȥDMA���е�����������
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //�����յ������ݴ�������
//    printf("UART6:%x,%x,%x,%x,%x\r\n",debugRvAll[0],debugRvAll[1],debugRvAll[2],debugRvAll[3],debugRvAll[4]);
    USART_PID_Adjust(1,DataBuff_UP);
    memset(&DataBuff_UP,0,data_length); //������ջ�����

    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DataBuff_UP, 200);   // ����UART�����ж�

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
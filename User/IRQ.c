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
        // UART_num=2;
        RxLine++;                            // ����������1
        DataBuff[RxLine-1]=RxBuffer[0];       // �����յ������ݴ��뻺����
        if(RxBuffer[0]=='!')                  // �ж��Ƿ���յ���̾��
        {
            printf("RXLen=%d\r\n",RxLine);
            // for(int i=0;i<RxLine;i++)
            //     printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
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
    // __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    // HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//ʹ�ܴ���6�Ŀ����ж�,���ڴ��ڽ���
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataBuff_UP, 200);//�������ڵ�DMA���գ�debugRvAll�洢���ڽ��ܵĵ�һ������

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{

    if(huart->Instance == huart3.Instance)//�ж��Ƿ��Ǵ���3
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//��������жϱ�־����ֹ��һֱ���Ͻ����ж�

            DMA_UP_UartIdleCallback(huart);//�����жϴ�����

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
    // imu600_parse(&debugRvAll[3]);
    memset(&debugRvAll,0,data_length); //������ջ�����

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//ѭ���п������ڵ�DMA����

}
void DMA_UP_UartIdleCallback(UART_HandleTypeDef *huart)//ע��һ�����⣬���õ�ʱ����д&huart6�����������������������
{
    HAL_UART_DMAStop(huart);//ֹͣ����DMA����

    //������յ������ݳ��ȣ����յ������ݳ��ȵ�����������洢���ȼ�ȥDMA���е�����������
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //�����յ������ݴ�������
    // printf("UART3");
    USART_PID_Adjust(1,DataBuff_UP);
    memset(&DataBuff_UP,0,data_length); //������ջ�����
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
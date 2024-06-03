
#include "main.h"
#include "PID_Adjust.h"
#include "PID_Control.h"
extern uint8_t RxBuffer[1];//���ڽ��ջ���
extern uint8_t DataBuff[200];//ָ������
extern PID pid_speed_A;
extern PID pid_position_A;
float Target_Speed;
float Target_Position;


/*
 * ������DataBuff�е�����
 * ���ؽ����õ�������
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0; // ��¼����λ�����ĵط�
    uint8_t data_Num = 0; // ��¼����λ��
    uint8_t minus_Flag = 0; // �ж��ǲ��Ǹ���
    float data_return = 0; // �����õ�������
    for(uint8_t i=0;i<200;i++) // ���ҵȺź͸�̾�ŵ�λ��
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // ����Ǹ���
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1; // ����flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // ���ݹ�4λ
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                      (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // ���ݹ�5λ
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                      (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // ���ݹ�6λ
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                      (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}

/*
 * ���ݴ�����Ϣ����PID����
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // ��Ž��յ�������
//    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 1)//��ߵ��
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // λ�û�P
            pid_position_A.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // λ�û�I
            pid_position_A.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // λ�û�D
            pid_position_A.kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // �ٶȻ�P
            pid_speed_A.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // �ٶȻ�I
            pid_speed_A.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // �ٶȻ�D
            pid_speed_A.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
            Target_Speed = data_Get;
        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //Ŀ��λ��
            Target_Position = data_Get*360;

    }
//    else if(Motor_n == 0) // �ұߵ��
//    {
//        if(DataBuff[0]=='P' && DataBuff[1]=='1') // λ�û�P
//            pid_r_position.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // λ�û�I
//            pid_r_position.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // λ�û�D
//            pid_r_position.kd = data_Get;
//        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // �ٶȻ�P
//            pid_r_speed.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // �ٶȻ�I
//            pid_r_speed.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // �ٶȻ�D
//            pid_r_speed.kd = data_Get;
//        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
//            R_Target_Speed = data_Get;
//        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //Ŀ��λ��
//            R_Target_Position = data_Get;
//    }
}



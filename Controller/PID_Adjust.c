


#include "all.h"
extern uint8_t RxBuffer[1];//���ڽ��ջ���
extern uint8_t DataBuff[200];//ָ������
extern PID pid_speed_A;
extern PID pid_position_A;
extern PID pid_angle;
float Target_Speed;
float Target_Position;
float Target_Angle;
float Target_Angle_actual;

/*
 * ������DataBuff�е�����
 * ���ؽ����õ�������
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0;   // ��¼����λ�����ĵط�
    uint8_t minus_Flag = 0;     // �ж��ǲ��Ǹ���
    float data_return = 0.0f;   // �����õ�������

    // ���ҵȺź͸�̾�ŵ�λ��
    for (uint8_t i = 0; i < 200; i++)
    {
        if (DataBuff[i] == '=')
            data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if (DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }

    // �жϸ���
    if (DataBuff[data_Start_Num] == '-')
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1;      // ����flag
    }

    // ��������
    char numberStr[20]; // ������󳤶�Ϊ19������С����ͽ�������
    uint8_t index = 0;

    for (uint8_t i = data_Start_Num; i <= data_End_Num && index < 19; i++)
    {
        if (DataBuff[i] == '\0') break; // �����ַ���������
        numberStr[index++] = DataBuff[i];
    }
    numberStr[index] = '\0'; // ����ַ���������

    // ʹ�� strtof ���ַ���ת��Ϊ������
    data_return = strtof(numberStr, NULL);

    if (minus_Flag == 1)
        data_return = -data_return;

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
            pid_angle.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // λ�û�I
            pid_angle.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // λ�û�D
            pid_angle.kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // �ٶȻ�P
            pid_speed_A.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // �ٶȻ�I
            pid_speed_A.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // �ٶȻ�D
            pid_speed_A.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
            servo2_angle = data_Get;
        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //Ŀ��λ��
            servo1_angle = data_Get;
        else if(DataBuff[0]=='R' && DataBuff[1]=='C'){
            if (sscanf(DataBuff, "RC=%d,%d", &L_TICK[0], &L_TICK[1]) == 2) {
                printf("���������%d �� %d\n", L_TICK[0], L_TICK[1]);
            } else {
                printf("��ʽ����\n");
            }
        }

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






#include "all.h"
extern uint8_t RxBuffer[1];//���ڽ��ջ���
extern uint8_t Databuff[200];//ָ������
extern PID pid_speed_A;
extern PID pid_position_A;
extern PID pid_angle;
float Target_Speed;
float Target_Position;
float Target_Angle;
float Target_Angle_actual;

uint16_t RxLine_UP;//指令长度
uint8_t RxBuffer_UP[1];//串口接收缓冲
uint8_t DataBuff_UP[200];//指令内容

/*
 * ������DataBuff�е�����
 * ���ؽ����õ�������
 */
float Get_Data(uint8_t* Data_Usart)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0;   // ��¼����λ�����ĵط�
    uint8_t minus_Flag = 0;     // �ж��ǲ��Ǹ���
    float data_return = 0.0f;   // �����õ�������

    // ���ҵȺź͸�̾�ŵ�λ��
    for (uint8_t i = 0; i < 200; i++)
    {
        if (Data_Usart[i] == '=')
            data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if (Data_Usart[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }

    // �жϸ���
    if (Data_Usart[data_Start_Num] == '-')
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1;      // ����flag
    }

    // ��������
    char numberStr[20]; // ������󳤶�Ϊ19������С����ͽ�������
    uint8_t index = 0;

    for (uint8_t i = data_Start_Num; i <= data_End_Num && index < 19; i++)
    {
        if (Data_Usart[i] == '\0') break; // �����ַ���������
        numberStr[index++] = Data_Usart[i];
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
void USART_PID_Adjust(uint8_t Motor_n,uint8_t *Data_buff)
{
    float data_Get = Get_Data(Data_buff); // ��Ž��յ�������
//    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 1)//��ߵ��
    {
        if(Data_buff[0]=='P' && Data_buff[1]=='1') // λ�û�P
            pid_angle.kp = data_Get;
        else if(Data_buff[0]=='I' && Data_buff[1]=='1') // λ�û�I
            pid_angle.ki = data_Get;
        else if(Data_buff[0]=='D' && Data_buff[1]=='1') // λ�û�D
            pid_angle.kd = data_Get;
        else if(Data_buff[0]=='P' && Data_buff[1]=='2') // �ٶȻ�P
            pid_speed_A.kp = data_Get;
        else if(Data_buff[0]=='I' && Data_buff[1]=='2') // �ٶȻ�I
            pid_speed_A.ki = data_Get;
        else if(Data_buff[0]=='D' && Data_buff[1]=='2') // �ٶȻ�D
            pid_speed_A.kd = data_Get;
        else if((Data_buff[0]=='S' && Data_buff[1]=='p') && Data_buff[2]=='e') //Ŀ���ٶ�
            servo2_angle = data_Get;
        else if((Data_buff[0]=='P' && Data_buff[1]=='o') && Data_buff[2]=='s') //Ŀ��λ��
            servo1_angle = data_Get;
        else if(Data_buff[0]=='R' && Data_buff[1]=='C'){
            if (sscanf(Data_buff, "RC=%d,%d", &L_TICK[0], &L_TICK[1]) == 2) {
                printf("���������%d �� %d\n", L_TICK[0], L_TICK[1]);
            } else {
                printf("��ʽ����\n");
            }

        }
        else if(Data_buff[0]=='P' && Data_buff[1]=='G'){//PG vx,vy,angle  !
            if(sscanf(Data_buff, "PG%f,%f,%f!", &Base_target_status.vy,&Base_target_status.vx, &Base_target_status.omega) == 3){
                Base_target_status.vy=-Base_target_status.vy;
//                printf("���������%.2f, %.2f, %.2f\n", Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
                Angle_Enable=0;
            }else{
//                printf("��ʽ����\n");
            }
        }
        else if(Data_buff[0]=='S' && Data_buff[1]=='R'){//SR servo1_angle,servo2_angle !
            if(sscanf(Data_buff, "SR%f,%f!", &servo1_angle,&servo2_angle) == 2){
//                printf("���������%.2f, %.2f\n", servo1_angle, servo2_angle);
            }else{
//                printf("��ʽ����\n");
            }
        }
        else if(Data_buff[0]=='A' && Data_buff[1]=='G'){//PG vx,vy,angle  !
            if(sscanf(Data_buff, "AG%f!", &Base_target_status.theta) == 1){
//                printf("���������%.2f\n", Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
                Angle_Enable=1;
            }else{
//                printf("��ʽ����\n");
            }
        }
    }
//    else if(Motor_n == 0) // �ұߵ��
//    {
//        if(Data_buff[0]=='P' && Data_buff[1]=='1') // λ�û�P
//            pid_r_position.kp = data_Get;
//        else if(Data_buff[0]=='I' && Data_buff[1]=='1') // λ�û�I
//            pid_r_position.ki = data_Get;
//        else if(Data_buff[0]=='D' && Data_buff[1]=='1') // λ�û�D
//            pid_r_position.kd = data_Get;
//        else if(Data_buff[0]=='P' && Data_buff[1]=='2') // �ٶȻ�P
//            pid_r_speed.kp = data_Get;
//        else if(Data_buff[0]=='I' && Data_buff[1]=='2') // �ٶȻ�I
//            pid_r_speed.ki = data_Get;
//        else if(Data_buff[0]=='D' && Data_buff[1]=='2') // �ٶȻ�D
//            pid_r_speed.kd = data_Get;
//        else if((Data_buff[0]=='S' && Data_buff[1]=='p') && Data_buff[2]=='e') //Ŀ���ٶ�
//            R_Target_Speed = data_Get;
//        else if((Data_buff[0]=='P' && Data_buff[1]=='o') && Data_buff[2]=='s') //Ŀ��λ��
//            R_Target_Position = data_Get;
//    }
}

void USART_From_UP()
{
    float data_Get = Get_Data(DataBuff_UP); // ��Ž��յ�������

    if((DataBuff_UP[0]=='S' && DataBuff_UP[1]=='p') && DataBuff_UP[2]=='e') //Ŀ���ٶ�
        servo2_angle = data_Get;
    else if((DataBuff_UP[0]=='P' && DataBuff_UP[1]=='o') && DataBuff_UP[2]=='s') //Ŀ��λ��
        servo1_angle = data_Get;

}

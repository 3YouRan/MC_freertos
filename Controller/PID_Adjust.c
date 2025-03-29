


#include "all.h"
extern uint8_t RxBuffer[1];//串口接收缓冲
extern uint8_t DataBuff[200];//指令内容
extern PID pid_speed_A;
extern PID pid_position_A;
extern PID pid_angle;
float Target_Speed;
float Target_Position;
float Target_Angle;
float Target_Angle_actual;

/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0;   // 记录数据位结束的地方
    uint8_t minus_Flag = 0;     // 判断是不是负数
    float data_return = 0.0f;   // 解析得到的数据

    // 查找等号和感叹号的位置
    for (uint8_t i = 0; i < 200; i++)
    {
        if (DataBuff[i] == '=')
            data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if (DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }

    // 判断负数
    if (DataBuff[data_Start_Num] == '-')
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1;      // 负数flag
    }

    // 解析数据
    char numberStr[20]; // 假设最大长度为19（包括小数点和结束符）
    uint8_t index = 0;

    for (uint8_t i = data_Start_Num; i <= data_End_Num && index < 19; i++)
    {
        if (DataBuff[i] == '\0') break; // 遇到字符串结束符
        numberStr[index++] = DataBuff[i];
    }
    numberStr[index] = '\0'; // 添加字符串结束符

    // 使用 strtof 将字符串转换为浮点数
    data_return = strtof(numberStr, NULL);

    if (minus_Flag == 1)
        data_return = -data_return;

    return data_return;
}

/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
//    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 1)//左边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            pid_angle.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            pid_angle.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            pid_angle.kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            pid_speed_A.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            pid_speed_A.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            pid_speed_A.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
            servo2_angle = data_Get;
        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
            servo1_angle = data_Get;
        else if(DataBuff[0]=='R' && DataBuff[1]=='C'){
            if (sscanf(DataBuff, "RC=%d,%d", &L_TICK[0], &L_TICK[1]) == 2) {
                printf("解析结果：%d 和 %d\n", L_TICK[0], L_TICK[1]);
            } else {
                printf("格式错误\n");
            }
        }

    }
//    else if(Motor_n == 0) // 右边电机
//    {
//        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
//            pid_r_position.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
//            pid_r_position.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
//            pid_r_position.kd = data_Get;
//        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
//            pid_r_speed.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
//            pid_r_speed.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
//            pid_r_speed.kd = data_Get;
//        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
//            R_Target_Speed = data_Get;
//        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
//            R_Target_Position = data_Get;
//    }
}



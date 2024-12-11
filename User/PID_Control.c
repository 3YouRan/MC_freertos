//
// Created by ��� on 24-6-2.
//

#include "PID_Control.h"
#include "FreeRTOS.h"
#include "task.h"
//PID�ṹ��
PID pid_speed_A;
PID pid_position_A;

PID pid_speed_B;
PID pid_position_B;

PID pid_speed_C;
PID pid_position_C;

PID pid_speed_D;
PID pid_position_D;

PID pid_angle;

extern float Target_Speed;
extern float Target_Position;
extern QueueHandle_t g_xPS2QueueHandle; //PS2�ֱ����о��
extern motor motorA, motorB, motorC, motorD;

void PID_Init(){
    pid_speed_A.err = 0;
    pid_speed_A.integral = 0;
    pid_speed_A.maxIntegral = 5000;
    pid_speed_A.maxOutput=1000;
    pid_speed_A.lastErr = 0;
    pid_speed_A.output = 0;
    pid_speed_A.kp = 5.07;
    pid_speed_A.ki = 0;
    pid_speed_A.kd = 12;
    pid_speed_A.deadZone = 0.01;

    pid_speed_B.err = 0;
    pid_speed_B.integral = 0;
    pid_speed_B.maxIntegral = 5000;
    pid_speed_B.maxOutput=1000;
    pid_speed_B.lastErr = 0;
    pid_speed_B.output = 0;
    pid_speed_B.kp = 5.07;
    pid_speed_B.ki = 0;
    pid_speed_B.kd = 12;
    pid_speed_B.deadZone = 0.01;

    pid_speed_C.err = 0;
    pid_speed_C.integral = 0;
    pid_speed_C.maxIntegral = 5000;
    pid_speed_C.maxOutput=1000;
    pid_speed_C.lastErr = 0;
    pid_speed_C.output = 0;
    pid_speed_C.kp = 5.07;
    pid_speed_C.ki = 0;
    pid_speed_C.kd = 12;
    pid_speed_C.deadZone = 0.01;

    pid_speed_D.err = 0;
    pid_speed_D.integral = 0;
    pid_speed_D.maxIntegral = 5000;
    pid_speed_D.maxOutput=1000;
    pid_speed_D.lastErr = 0;
    pid_speed_D.output = 0;
    pid_speed_D.kp = 5.07;
    pid_speed_D.ki = 0;
    pid_speed_D.kd = 12;
    pid_speed_D.deadZone = 0.01;

    pid_angle.err = 0;
    pid_angle.integral = 0;
    pid_angle.maxIntegral = 5000;
    pid_angle.maxOutput=1000;
    pid_angle.lastErr = 0;
    pid_angle.output = 0;
    pid_angle.kp = 10;
    pid_angle.ki = 0;
    pid_angle.kd = 0;
    pid_angle.deadZone = 0.01;
}
/****************************************
 * ���ã�����ʽPID
 * ������PID�����ṹ���ַ��Ŀ��ֵ������ֵ
 * ����ֵ��PID���ֵ
 * ****************************************/
float INC_PID_Realize(PID* pid,float target,float feedback)//һ��PID����
{
    pid->err = target - feedback;
    if(pid->err < pid->deadZone && pid->err > -pid->deadZone) pid->err = 0;//pid����
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//�����޷�
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // ɲ��ʱ���i


    pid->output += (pid->kp * pid->err) + (pid->ki * pid->integral)
                   + (pid->kd * (pid->err - pid->lastErr));//����ʽPID

    //����޷�
    if(target >= 0)//��תʱ
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//��תʱ
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // ɲ��ʱֱ�����0
    return pid->output;

}
/****************************************
 * ���ã�ȫ��ʽPID
 * ������PID�����ṹ���ַ��Ŀ��ֵ������ֵ
 * ����ֵ��PID���ֵ
 * ****************************************/
float FULL_PID_Realize(PID* pid,float target,float feedback)//һ��PID����
{
    pid->err = target - feedback;
    if(pid->err < pid->deadZone && pid->err > -pid->deadZone) pid->err = 0;//pid����
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//�����޷�
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // ɲ��ʱ���i

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral)
                   + (pid->kd * (pid->err - pid->lastErr));//ȫ��ʽPID

//    //����޷�
//    if(target >= 0)//��תʱ
//    {
//        if(pid->output < 0) pid->output = 0;
//        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
//    }
//    else if(target < 0)//��תʱ
//    {
//        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
//        else if(pid->output > 0) pid->output = 0;
//    }

    pid->lastErr = pid->err;
//    if(target == 0) pid->output = 0; // ɲ��ʱֱ�����0
    return pid->output;

}
extern uint8_t Key1;
extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;
extern float Target_Speed_A_Now;
extern float Target_Speed_B_Now;
extern float Target_Speed_C_Now;
extern float Target_Speed_D_Now;
extern float Target_Angle;
extern float angle_Car;
uint8_t direction ;
float angle_speed = 0;
float Vx=0;
float Vy=0;
uint8_t mode_flag=0;//ģʽ1��ң�أ�ģʽ0��ѭ��
extern uint8_t sensor[4];
void LED_Trun_right(void){//��ת���
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);

}
void LED_Trun_left(void){//��ת���
    HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

}
void Base_Control(void *argument){

    while(1){
        if(angle_speed>0){//��ת����ת�����
            LED_Trun_right();
        }
        else if(angle_speed<0){//��ת����ת�����
            LED_Trun_left();
        }
        if(mode_flag==0){
            switch (Key1) {

                case 12:
                    mode_flag = 1;//�л�Ϊѭ��ģʽ
                case 13:
                    Vx = 0;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 14:
                    Vx = -30;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 15:
                    Vx = -60;
                    break;
                case 16:
                    Vx = -150;
                    break;

            }

            Target_Angle=angle_Car;//ʧ�ܽǶȻ�
            sensor[0]=HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin);
            sensor[1]=HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin);
            sensor[2]=HAL_GPIO_ReadPin(R1_GPIO_Port,R1_Pin);
            sensor[3]=HAL_GPIO_ReadPin(R2_GPIO_Port,R2_Pin);

            if(sensor[0]==0&&sensor[1]==1&&sensor[2]==1&&sensor[3]==0){//�м�

                angle_speed=0;
            }
            if(sensor[0]==1&&sensor[1]==1&&sensor[2]==0&&sensor[3]==0){//ƫ��

                angle_speed=200;
            }
            if(sensor[0]==0&&sensor[1]==1&&sensor[2]==0&&sensor[3]==0){//ƫ��
                angle_speed=200;
            }
            if(sensor[0]==1&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0){//ƫ��
                angle_speed=400;
            }

            if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==1){//ƫ��
                angle_speed=-200;
            }
            if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==0){//ƫ��
                angle_speed=-200;
            }
            if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==1){//ƫ��
                angle_speed=-400;
            }
            Kinematic_Analysis(Vx,Vy,-angle_speed);
        }//ѭ��ģʽ����������
        else if(mode_flag==1) {
            switch (Key1) {
                case 5:
                    direction = FORWARD;
                    break;//ǰ������??
                case 6:
                    direction = RIGHT;
                    break;//ǰ������??
                case 7:
                    direction = BACK;
                    break;//ǰ������??
                case 8:
                    direction = LEFT;
                    break;//ǰ������??
                case 10:
                    Target_Angle += -5;
                    break;//L2��ʹС����ʱ����??90??
                case 9:
                    Target_Angle += 5;
                    break;//R2��ʹС����ʱ����??90??
                case 12:
                    mode_flag = 0;//�л�Ϊѭ��ģʽ
                case 13:
                    Target_Speed = 0;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 14:
                    Target_Speed = 30;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 15:
                    Target_Speed = 60;
                    break;
                case 16:
                    Target_Speed = 150;
                    break;
                    //default:Target_Speed=0;break;
            }
            switch (direction) {
                case FORWARD:

                    Vx = -Target_Speed;
                    Vy = 0;
                    break;
                case RIGHT:

                    Vx = 0;
                    Vy = Target_Speed;
                    break;
                case BACK:

                    Vx = Target_Speed;
                    Vy = 0;
                    break;
                case LEFT:

                    Vx = 0;
                    Vy = -Target_Speed;
                    break;
            }
            Kinematic_Analysis(Vx,Vy,-angle_speed);
        }//ң��ģʽ��ң��������

        vTaskDelay(100);
    }
}
extern float angle_Car_total;
void PID_Task(void *argument){
    portTickType CurrentTime_PID;
    while(1){
        CurrentTime_PID=xTaskGetTickCount();
        //����ٶ�PID����
        INC_PID_Realize(&pid_speed_A, Target_Speed_A, motorA.speed);
        INC_PID_Realize(&pid_speed_B, -Target_Speed_B, motorB.speed);
        INC_PID_Realize(&pid_speed_C, Target_Speed_C, motorC.speed);
        INC_PID_Realize(&pid_speed_D, -Target_Speed_D, motorD.speed);
        //�ǶȻ�
        angle_speed = FULL_PID_Realize(&pid_angle, Target_Angle,angle_Car_total);

        //���PID������
        motorA_run(pid_speed_A.output);
        motorB_run(pid_speed_B.output);
        motorC_run(pid_speed_C.output);
        motorD_run(pid_speed_D.output);

        vTaskDelayUntil(&CurrentTime_PID,5);
    }
}
//void PID_Timer_Callback(TimerHandle_t pxTimer ) {
//    while (1) {
//        INC_PID_Realize(&pid_speed_A, Target_Speed, motorA.speed);
//        INC_PID_Realize(&pid_speed_B, -Target_Speed, motorB.speed);
//        INC_PID_Realize(&pid_speed_C, Target_Speed, motorC.speed);
//        INC_PID_Realize(&pid_speed_D, -Target_Speed, motorD.speed);
//
//        motorA_run(pid_speed_A.output);
//        motorB_run(pid_speed_B.output);
//        motorC_run(pid_speed_C.output);
//        motorD_run(pid_speed_D.output);
//    }
//}
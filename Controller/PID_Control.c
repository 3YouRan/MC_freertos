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
    pid_angle.kp = 15;
    pid_angle.ki = 0;
    pid_angle.kd = 0;
    pid_angle.deadZone = 0.2f;
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

void LED_Trun_right(void){//��ת���
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);

}
void LED_Trun_left(void){//��ת���
    HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

}


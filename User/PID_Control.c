//
// Created by 陈瑜 on 24-6-2.
//

#include "PID_Control.h"

//PID结构体
PID pid_speed_A;
PID pid_position_A;

PID pid_speed_B;
PID pid_position_B;

PID pid_speed_C;
PID pid_position_C;

PID pid_speed_D;
PID pid_position_D;

extern float Target_Speed;
extern float Target_Position;
extern QueueHandle_t g_xPS2QueueHandle; //PS2手柄队列句柄
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
}
/****************************************
 * 作用：增量式PID
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：PID输出值
 * ****************************************/
float INC_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < pid->deadZone && pid->err > -pid->deadZone) pid->err = 0;//pid死区
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i


    pid->output += (pid->kp * pid->err) + (pid->ki * pid->integral)
                   + (pid->kd * (pid->err - pid->lastErr));//增量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
    return pid->output;

}
/****************************************
 * 作用：全量式PID
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：PID输出值
 * ****************************************/
float FULL_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < pid->deadZone && pid->err > -pid->deadZone) pid->err = 0;//pid死区
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral)
                   + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
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

uint8_t direction ;
void Base_Control(void *argument){

    while(1){

        switch (Key1) {
            case 5:direction=FORWARD;break;//前进方向??
            case 6:direction=RIGHT;break;//前进方向??
            case 7:direction=BACK;break;//前进方向??
            case 8:direction=LEFT;break;//前进方向??
//            case 10:Target_Angle+=-5;
//                break;//L2键使小车逆时针旋??90??
//            case 9:Target_Angle+=5;
//                break;//R2键使小车逆时针旋??90??
            case 13:Target_Speed = 0;break;//速度档位调节，rpm
            case 14:Target_Speed = 30;break;//速度档位调节，rpm
            case 15:Target_Speed = 60;break;
            case 16:Target_Speed=150;break;
                //default:Target_Speed=0;break;
        }
        switch (direction) {
            case FORWARD:
                Kinematic_Analysis(-Target_Speed,0,0);
                break;
            case RIGHT:
                Kinematic_Analysis(0,Target_Speed,0);
                break;
            case BACK:
                Kinematic_Analysis(Target_Speed,0,0);
                break;
            case LEFT:
                Kinematic_Analysis(0,-Target_Speed,0);
                break;
        }

        vTaskDelay(100);
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
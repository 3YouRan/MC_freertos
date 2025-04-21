//
// Created by 陈瑜 on 25-3-22.
//


#include "all.h"
uint8_t Motor_Enable = 1;
uint16_t pid_delay = 0;
uint8_t pid_flag = 0;

float Target_Speed_A=0;
float Target_Speed_B=0;
float Target_Speed_C=0;
float Target_Speed_D=0;
float Target_Speed_A_Now=0;
float Target_Speed_B_Now=0;
float Target_Speed_C_Now=0;
float Target_Speed_D_Now=0;

void PID_Task(void *argument){
    portTickType CurrentTime_PID;
    while(1){
        CurrentTime_PID=xTaskGetTickCount();
        if(pid_flag==0){
            pid_delay ++;
            if (pid_delay >= 200){// 500ms
                pid_flag = 1;
            }
        }else if(pid_flag == 1&&Motor_Enable==1) {
            //电机速度PID控制
            INC_PID_Realize(&pid_speed_A, Target_Speed_A, motorA.speed);
            INC_PID_Realize(&pid_speed_B, -Target_Speed_B, motorB.speed);
            INC_PID_Realize(&pid_speed_C, Target_Speed_C, motorC.speed);
            INC_PID_Realize(&pid_speed_D, -Target_Speed_D, motorD.speed);
            //角度环
            Base_status.omega= -FULL_PID_Realize(&pid_angle, Target_Angle_actual, yaw_total);
        }
        //输出PID计算结果
        Motor_Enable = HAL_GPIO_ReadPin(Motor_Enable_GPIO_Port,Motor_Enable_Pin);
        if(Motor_Enable == 1) {
            motorA_run((int) pid_speed_A.output);
            motorB_run((int) pid_speed_B.output);
            motorC_run((int) pid_speed_C.output);
            motorD_run((int) pid_speed_D.output);
        } else{
            motorA_run((int) 0);
            motorB_run((int) 0);
            motorC_run((int) 0);
            motorD_run((int) 0);
        }
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
//
// Created by ��� on 25-3-22.
//


#include "all.h"
uint8_t Motor_Enable = 1;
uint16_t pid_delay = 0;
uint8_t pid_flag = 0;


void PID_Task(void *argument){
    portTickType CurrentTime_PID;
    while(1){
        CurrentTime_PID=xTaskGetTickCount();
        if(pid_flag==0){
            pid_delay ++;
            if (pid_delay >= 200){// 500ms
                pid_flag = 1;
            }
        }else if(pid_flag == 1) {
            //����ٶ�PID����
            INC_PID_Realize(&pid_speed_A, motorA.TargetSpeed, motorA.speed);
            INC_PID_Realize(&pid_speed_B, motorB.TargetSpeed, motorB.speed);
            INC_PID_Realize(&pid_speed_C, motorC.TargetSpeed, motorC.speed);
            INC_PID_Realize(&pid_speed_D, motorD.TargetSpeed, motorD.speed);
            //�ǶȻ�
            Base_target_status.omega= FULL_PID_Realize(&pid_angle, Base_target_status.theta, yaw_total);
        }
        //�жϵ��ʹ��

        //���PID������
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
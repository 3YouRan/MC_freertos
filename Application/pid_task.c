//
// Created by ��� on 25-3-22.
//


#include "all.h"
uint8_t Motor_Enable = 1;
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
        Motor_Enable = HAL_GPIO_ReadPin(Motor_Enable_GPIO_Port,Motor_Enable_Pin);
        if(Motor_Enable == 1) {
            motorA_run((int) pid_speed_A.output);
            motorB_run((int) pid_speed_B.output);
            motorC_run((int) pid_speed_C.output);
            motorD_run((int) pid_speed_D.output);
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
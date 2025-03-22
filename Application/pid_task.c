//
// Created by 陈瑜 on 25-3-22.
//


#include "all.h"

void PID_Task(void *argument){
    portTickType CurrentTime_PID;
    while(1){
        CurrentTime_PID=xTaskGetTickCount();
        //电机速度PID控制
        INC_PID_Realize(&pid_speed_A, Target_Speed_A, motorA.speed);
        INC_PID_Realize(&pid_speed_B, -Target_Speed_B, motorB.speed);
        INC_PID_Realize(&pid_speed_C, Target_Speed_C, motorC.speed);
        INC_PID_Realize(&pid_speed_D, -Target_Speed_D, motorD.speed);
        //角度环
        angle_speed = FULL_PID_Realize(&pid_angle, Target_Angle,angle_Car_total);

        //输出PID计算结果
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
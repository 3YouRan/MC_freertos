//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"
void UART6_Task(void *argument){
    while(1){
//        printf("UART2:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n\r",pid_angle.kp,pid_angle.output,Target_Angle,motorA.speed,angle_Car_total,angle_speed,Key1);
        vTaskDelay(55);
        printf("Angle:%.2f\r\n",92.33);
    }
}
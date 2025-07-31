//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"


void UART6_Task(void *argument){
    while(1){
        printf("UART:%d,%.2f,%.2f,%d,%d\r\n",shoot_flag,motor_angle1,motor_angle2,Pulse_num1,Pulse_num2);
        vTaskDelay(109);


    }
}
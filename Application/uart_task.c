//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"


void UART6_Task(void *argument){
    while(1){

        vTaskDelay(100);

        printf("UART6:%.2f,%.2f,%.2f,%.2f,%.2f\n\r",Base_target_status.vx,Base_target_status.vy,Base_target_status.omega,servo1_angle,servo2_angle);


    }
}
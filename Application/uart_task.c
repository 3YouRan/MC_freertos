//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"


void UART6_Task(void *argument){
    while(1){

        vTaskDelay(50);

        printf("%.2f\n\r",yaw_total);


    }
}
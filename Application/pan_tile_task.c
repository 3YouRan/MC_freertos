//
// Created by ³Âè¤ on 25-3-23.
//

#include "all.h"
float servo1_angle = 90;
float servo2_angle = 130;
uint8_t hit_flag=0;
void pan_tile_task(void *arg) {
    while (1) {
        servo1_set(servo1_angle);
        servo2_set(servo2_angle);

        if(hit_flag==1){
            servo3_set(180);
            hit_flag=0;
            vTaskDelay(1000);
        } else{
            servo3_set(0);
        }

        vTaskDelay(50);
    }
}
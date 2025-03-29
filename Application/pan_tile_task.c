//
// Created by ³Âè¤ on 25-3-23.
//

#include "all.h"
float servo1_angle = 90;
float servo2_angle = 90;

void pan_tile_task(void *arg) {
    while (1) {
        servo1_set(servo1_angle);
        servo2_set(servo2_angle);
//        SERVO1_SET(50);
        vTaskDelay(50);
    }
}
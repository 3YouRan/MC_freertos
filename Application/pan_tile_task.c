//
// Created by ��� on 25-3-23.
//

#include "all.h"
float servo1_angle = 90;
float servo2_angle = 130;

void pan_tile_task(void *arg) {
    while (1) {
        servo1_set(servo1_angle);
        servo2_set(servo2_angle);
      //  SERVO2_SET(2500);
//        SERVO1_SET(50);
        vTaskDelay(50);
    }
}
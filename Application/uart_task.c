//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"


void UART6_Task(void *argument){
    while(1){

        vTaskDelay(50);

//        printf("UART6:%.2f,%.2f,%.2f,%.2f,%.2f\n\r",Base_target_status.vx,Base_target_status.vy,Base_target_status.omega,servo1_angle,servo2_angle);
//        printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
        printf("UART:%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", back_x_flag,back_y_flag, Base_target_status.vx, Base_target_status.vy, Base_target_status.omega,Base_target_status.theta, yaw_total, ax_imu, ay_imu, Base_odometry.x, Base_odometry.y);

//
    }
}
//
// Created by ³Âè¤ on 25-3-22.
//


#include "all.h"


void UART6_Task(void *argument){
    while(1){
//        printf("UART6:%d,%d,%d,%d,%.2f,%.2f\r\n",L_TICK[0],L_TICK[1],R_TICK[0],R_TICK[1],Vx,Vy);
//        printf("UART6:%d,%.2f,%.2f,%.2f\r\n",1,(float) stcAngle.Angle[2] / 32768 * 180,yaw_offset,yaw_total);
        vTaskDelay(50);
//        printf("PS2:%d,%d,%d,%d,%d\r\n",Key1,L_TICK[0],L_TICK[1],R_TICK[0],R_TICK[1]);

//        printf("1:%d,%d,%d,%d,%.5f\r\n",V_A[0],V_A[1],V_A[2],Motor_Enable,ADC_Value[0]/4096.0*5*11);
//        printf("UART6:%.2f,%.2f,%.2f,%.2f\r\n",Target_Speed_A_Now,Target_Speed_B_Now,Target_Speed_C_Now,Target_Speed_D_Now);
//          printf("UART6:%d,%d,%d,%d\r\n",(int16_t)motorA.speed,(int16_t)motorB.speed,(int16_t)motorC.speed,(int16_t)motorD.speed);
//        printf("Angle:%.2f\r\n",angle_speed);
        printf("UART6:%.2f\r\n",servo1_angle);
    }
}
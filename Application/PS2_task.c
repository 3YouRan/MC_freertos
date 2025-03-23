//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"//PS2手柄任务
uint8_t L_TICK[2] = {0, 0};//左摇杆数据
uint8_t R_TICK[2] = {0, 0};//右摇杆数据
void PS2_Task(void *argment){

    while(1){
        PS2_ReadData();          //获取数据
        Key1 = PS2_DataKey();       //获取手柄按键数据
        L_TICK[0]= PS2_AnologData(PSS_LX);//获取左摇杆X轴数据
        L_TICK[1]= PS2_AnologData(PSS_LY);//获取左摇杆Y轴数据
        R_TICK[0]= PS2_AnologData(PSS_RX);//获取左摇杆X轴数据
        R_TICK[1]= PS2_AnologData(PSS_RY);//获取左摇杆Y轴数据
        PS2_ClearData();      //清除手柄按键数据
        vTaskDelay(10);
    }
}
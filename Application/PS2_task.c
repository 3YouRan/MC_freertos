//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"//PS2手柄任务
void PS2_Task(void *argment){

    while(1){
        PS2_ReadData();          //获取数据
        Key1 = PS2_DataKey();       //获取手柄按键数据
        PS2_ClearData();      //清除手柄按键数据
        vTaskDelay(20);
    }
}
//
// Created by ��� on 25-3-22.
//

#include "all.h"//PS2�ֱ�����
void PS2_Task(void *argment){

    while(1){
        PS2_ReadData();          //��ȡ����
        Key1 = PS2_DataKey();       //��ȡ�ֱ���������
        PS2_ClearData();      //����ֱ���������
        vTaskDelay(20);
    }
}
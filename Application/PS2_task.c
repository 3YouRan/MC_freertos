//
// Created by ��� on 25-3-22.
//

#include "all.h"//PS2�ֱ�����
uint8_t L_TICK[2] = {0, 0};//��ҡ������
uint8_t R_TICK[2] = {0, 0};//��ҡ������
void PS2_Task(void *argment){

    while(1){
        PS2_ReadData();          //��ȡ����
        Key1 = PS2_DataKey();       //��ȡ�ֱ���������
        L_TICK[0]= PS2_AnologData(PSS_LX);//��ȡ��ҡ��X������
        L_TICK[1]= PS2_AnologData(PSS_LY);//��ȡ��ҡ��Y������
        R_TICK[0]= PS2_AnologData(PSS_RX);//��ȡ��ҡ��X������
        R_TICK[1]= PS2_AnologData(PSS_RY);//��ȡ��ҡ��Y������
        PS2_ClearData();      //����ֱ���������
        vTaskDelay(10);
    }
}
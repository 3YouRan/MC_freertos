//
// Created by ��� on 25-5-20.
//

#include "all.h"
uint8_t Buzzer_Flag = 0;
void BUZZER_Task(void *arg){
    while(1){
        if(Buzzer_Flag == BUZZER_STOP){
            Buzzer_On();
            Buzzer_Flag=0;
        }
        else if(Buzzer_Flag == BUZZER_FOUND){
            Buzzer_On();//�ҵ�Ŀ�������5��
            Buzzer_On();
            Buzzer_On();
            Buzzer_On();
            Buzzer_On();
            Buzzer_Flag=0;
        }
        vTaskDelay(200);
    }
}
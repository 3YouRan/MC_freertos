//
// Created by 陈瑜 on 24-6-1.
//

#include "all.h"
void Buzzer_On() {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    vTaskDelay(200);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);

}



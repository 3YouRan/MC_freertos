//
// Created by ��� on 25-3-23.
//

#include "all.h"
//���ö���Ƕ�:0-270�� 0:����0.5ms,180:����2.5ms PWM����20ms
uint16_t Ang_to_Pulse_Num(float angle) {
    uint16_t pulse_num = (angle/360.0f)*200.0f*32.0f; // �Ƕ�ת������

    return pulse_num;
}
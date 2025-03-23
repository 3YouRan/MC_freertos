//
// Created by ��� on 25-3-23.
//

#include "all.h"
//���ö���Ƕ�:0-270�� 0:����0.5ms,270:����2.5ms PWM����20ms
void servo1_set(float angle) {
    if(angle<0)//���ƽǶȷ�Χ
        angle=0;
    if(angle>270)
        angle=270;

    float width=angle/270.0f;//�Ƕ�ת�������
    width=width*(PW_MAX-PW_MIN)+PW_MIN;//�������ת����ֵ

    SERVO1_SET((uint16_t)width);

}
//���ö���Ƕ�:0-180�� 0:����0.5ms,180:����Լ1.83ms PWM����20ms
void servo2_set(float angle) {
    if(angle<0)//���ƽǶȷ�Χ
        angle=0;
    if(angle>180)
        angle=180;

    float width=angle/270.0f;//�Ƕ�ת�������
    width=width*(PW_MAX-PW_MIN)+PW_MIN;//�������ת����ֵ

    SERVO2_SET((uint16_t)width);

}
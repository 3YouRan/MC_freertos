//
// Created by ��� on 24-6-3.
//

#include "Kinematic_Analysis.h"
extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;

/**************************************************************************
�����ķ�����˶�ѧģ��
A B Ϊǰ����
C D Ϊ������
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    Target_Speed_A=-(Vx-Vy-V_angle*RxPLUSRy);//��ǰ
    Target_Speed_B=-(Vx+Vy+V_angle*RxPLUSRy);//��ǰ
    Target_Speed_C=-(Vx+Vy-V_angle*RxPLUSRy);//���
    Target_Speed_D=-(Vx-Vy+V_angle*RxPLUSRy);//�Һ�
}
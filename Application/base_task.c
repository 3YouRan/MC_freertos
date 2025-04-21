//
// Created by ��� on 25-3-22.
//
#include "all.h"

Base_status_t Base_status;

uint8_t mode_flag = 1;
uint8_t direction = FORWARD;
uint8_t delay_times = 0;
uint8_t delay_flag = 0;

void Base_Control(void *argument){

    while(1){
        if(delay_flag==0) {
            delay_times++;
        }
        if(delay_times>10){
            delay_flag = 1;
            delay_times = 0;
        }
        if(mode_flag==RC_MODE&&delay_flag==1) {//ң��ģʽ��ң��������
            //�ٶȿ���
            if((L_TICK[1]+L_TICK[0])>500){//δ����ң����ʱ��Ĭ��ֹͣ
                Base_status.vx = 0;
                Base_status.vy = 0;

            }else if(abs((L_TICK[0]-128)+(L_TICK[1]-128))>20){
                Base_status.vx = (L_TICK[0]-128)*5;
                Base_status.vy = (L_TICK[1]-128)*5;

            }else {
                Base_status.vx = 0;
                Base_status.vy = 0;
            }

            Kinematic_Analysis(Base_status.vx,Base_status.vy,Base_status.omega);
        }
        else if(mode_flag==SLAVE_MODE&&delay_flag==1) {//slaveģʽ����λ�����ڿ���

        }

        vTaskDelay(20);
    }
}

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
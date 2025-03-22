//
// Created by ��� on 25-3-22.
//
#include "all.h"
#include "base_task.h"
float Vx;
float Vy;
float angle_speed;
uint8_t mode_flag = 1;
uint8_t direction = FORWARD;

void Base_Control(void *argument){

    while(1){
        if(mode_flag==1) {//ң��ģʽ��ң��������
            switch (Key1) {
                case 5:
                    direction = FORWARD;
                    break;//ǰ������??
                case 6:
                    direction = RIGHT;
                    break;//ǰ������??
                case 7:
                    direction = BACK;
                    break;//ǰ������??
                case 8:
                    direction = LEFT;
                    break;//ǰ������??
                case 10:
                    Target_Angle += -5;
                    break;//L2��ʹС����ʱ����??90??
                case 9:
                    Target_Angle += 5;
                    break;//R2��ʹС����ʱ����??90??
                case 12:
                    mode_flag = 0;//�л�Ϊѭ��ģʽ
                case 13:
                    Target_Speed = 0;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 14:
                    Target_Speed = 30;
                    break;//�ٶȵ�λ���ڣ�rpm
                case 15:
                    Target_Speed = 60;
                    break;
                case 16:
                    Target_Speed = 150;
                    break;
                    //default:Target_Speed=0;break;
            }
            switch (direction) {
                case FORWARD:

                    Vx = -Target_Speed;
                    Vy = 0;
                    break;
                case RIGHT:

                    Vx = 0;
                    Vy = Target_Speed;
                    break;
                case BACK:

                    Vx = Target_Speed;
                    Vy = 0;
                    break;
                case LEFT:

                    Vx = 0;
                    Vy = -Target_Speed;
                    break;
            }
            Kinematic_Analysis(Vx,Vy,-angle_speed);
        }

        vTaskDelay(100);
    }
}
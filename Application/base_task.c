//
// Created by 陈瑜 on 25-3-22.
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
        if(mode_flag==1) {//遥控模式，遥控器控制
            switch (Key1) {
                case 5:
                    direction = FORWARD;
                    break;//前进方向??
                case 6:
                    direction = RIGHT;
                    break;//前进方向??
                case 7:
                    direction = BACK;
                    break;//前进方向??
                case 8:
                    direction = LEFT;
                    break;//前进方向??
                case 10:
                    Target_Angle += -5;
                    break;//L2键使小车逆时针旋??90??
                case 9:
                    Target_Angle += 5;
                    break;//R2键使小车逆时针旋??90??
                case 12:
                    mode_flag = 0;//切换为循迹模式
                case 13:
                    Target_Speed = 0;
                    break;//速度档位调节，rpm
                case 14:
                    Target_Speed = 30;
                    break;//速度档位调节，rpm
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
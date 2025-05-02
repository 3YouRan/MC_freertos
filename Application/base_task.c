//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"

Base_status_t Base_target_status;//底盘目标状态
OdometryState_t Base_odometry;//底盘里程计

uint8_t mode_flag = SLAVE_MODE;
uint8_t delay_times = 0;
uint8_t delay_flag = 0;

void Base_Control(void *argument){

    while(1){
        //判断电机是否使能
        Motor_Enable = HAL_GPIO_ReadPin(Motor_Enable_GPIO_Port,Motor_Enable_Pin);

        if(delay_flag==0) {
            delay_times++;
        }
        if(delay_times>10){
            delay_flag = 1;
            delay_times = 0;
        }
        if(mode_flag==RC_MODE&&delay_flag==1) {//遥控模式，遥控器控制
//            //速度控制
//            if((L_TICK[1]+L_TICK[0])>500){//未连接遥控器时，默认停止
//                Base_target_status.vx = 0;
//                Base_target_status.vy = 0;
//
//            }else if(abs((L_TICK[0]-128)+(L_TICK[1]-128))>20){
////                Base_target_status.vx = (L_TICK[0] - 128) * 5;
////                Base_target_status.vy = (L_TICK[1] - 128) * 5;
//
//            }else {
//                Base_target_status.vx = 0;
//                Base_target_status.vy = 0;
//            }
            Base_target_status.vx = 20;
//            Base_target_status.vy = 20;
            Kinematic_Analysis(Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
        }
        else if(mode_flag==SLAVE_MODE&&delay_flag==1) {//slave模式，上位机串口控制
            Kinematic_Analysis(Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
        }

        vTaskDelay(20);
    }
}

/**************************************************************************
麦克纳姆轮逆运动学模型
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    float Vx_robot= Vx*cos(yaw_total/360*2*M_PI) - Vy*sin(yaw_total/360*2*M_PI);
    float Vy_robot= Vx*sin(yaw_total/360*2*M_PI) + Vy*cos(yaw_total/360*2*M_PI);
    float V_angle_robot= V_angle;

    motorA.TargetSpeed=(Vx_robot-Vy_robot-V_angle_robot*RxPLUSRy);//左前
    motorB.TargetSpeed=(Vx_robot+Vy_robot+V_angle_robot*RxPLUSRy);//右前
    motorC.TargetSpeed=(Vx_robot+Vy_robot-V_angle_robot*RxPLUSRy);//左后
    motorD.TargetSpeed=(Vx_robot-Vy_robot+V_angle_robot*RxPLUSRy);//右后
}
void calculate_odometry(float w1, float w2, float w3, float w4,
                        OdometryState_t * state, float delta_time) {
    // 将轮子角速度转换为线速度（m/s）
    const float l = FRONT_TO_BACK_SIZE/2.0f;
    const float w = RIGHT_TO_LEFT_SIZE/2.0f;
    const float r = WHEEL_DIAMETER/2.0f;

    // 运动学矩阵系数
    const float inv_mat = 1.0f / (4.0f * r);

    // 计算底盘速度（逆运动学）
    state->vx = inv_mat * ( w1 + w2 + w3 + w4) * r;
    state->vy = inv_mat * (-w1 + w2 - w3 + w4) * r;
//    state->omega = inv_mat * (-w1 + w2 - w3 + w4) * r / (l + w);

    // 积分得到位置和航向
    state->x += (state->vx * cos(yaw_total/360*2*M_PI) - state->vy * sin(yaw_total/360*2*M_PI)) * delta_time;
    state->y += (state->vx * sin(yaw_total/360*2*M_PI) + state->vy * cos(yaw_total/360*2*M_PI)) * delta_time;
//    state->theta += state->omega * delta_time;

    // 角度归一化到[-π, π]
    if (state->theta > M_PI) {
        state->theta -= 2 * M_PI;
    } else if (state->theta < -M_PI) {
        state->theta += 2 * M_PI;
    }
}
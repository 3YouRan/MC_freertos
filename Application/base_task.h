//
// Created by 陈瑜 on 25-3-22.
//

#ifndef MC_PROJ_BASE_TASK_H
#define MC_PROJ_BASE_TASK_H



#define RC_MODE 0
#define Pos_MODE 1
#define SLAVE_MODE 2

#define RxPLUSRy (0.1097/2+0.1408/2)
#define FRONT_TO_BACK_SIZE 0.1408
#define RIGHT_TO_LEFT_SIZE 0.1097
#define WHEEL_DIAMETER 0.075

typedef struct {
    float vx;
    float vy;
    float omega;
    float x;
    float y;
    float theta;
    int srv_angle1;
    int srv_angle2;
    uint8_t mode;
}Base_status_t;//底盘目标状态

typedef struct {
    float x;         // X轴位置（米）
    float y;         // Y轴位置（米）
    float theta;     // 航向角（弧度）
    float vx;        // X轴速度（米/秒）
    float vy;        // Y轴速度（米/秒）
    float omega;     // 旋转角速度（弧度/秒）
} OdometryState_t;// 里程计状态结构体

void Base_Control(void *argument);
void Kinematic_Analysis(float Vx,float Vy,float V_angle);
void calculate_odometry(float w1, float w2, float w3, float w4,
                        OdometryState_t * state, float delta_time);
#endif //MC_PROJ_BASE_TASK_H

//
// Created by ³Âè¤ on 25-3-22.
//

#ifndef MC_PROJ_BASE_TASK_H
#define MC_PROJ_BASE_TASK_H



#define RC_MODE 0
#define Pos_MODE 1
#define SLAVE_MODE 2

#define RxPLUSRy (0.1097/2+0.1408/2)

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
}Base_status_t;//µ×ÅÌ×´Ì¬


void Base_Control(void *argument);
void Kinematic_Analysis(float Vx,float Vy,float V_angle);
#endif //MC_PROJ_BASE_TASK_H

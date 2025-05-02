//
// Created by ��� on 25-3-22.
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
}Base_status_t;//����Ŀ��״̬

typedef struct {
    float x;         // X��λ�ã��ף�
    float y;         // Y��λ�ã��ף�
    float theta;     // ����ǣ����ȣ�
    float vx;        // X���ٶȣ���/�룩
    float vy;        // Y���ٶȣ���/�룩
    float omega;     // ��ת���ٶȣ�����/�룩
} OdometryState_t;// ��̼�״̬�ṹ��

void Base_Control(void *argument);
void Kinematic_Analysis(float Vx,float Vy,float V_angle);
void calculate_odometry(float w1, float w2, float w3, float w4,
                        OdometryState_t * state, float delta_time);
#endif //MC_PROJ_BASE_TASK_H

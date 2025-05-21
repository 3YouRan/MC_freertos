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

// �������ά��
#define N_STATE 4   // ״̬�� [x, y, vx, vy]
#define N_CTRL 2    // ������ [ax, ay]
#define N_OBS 2     // �۲��� [x_odo, y_odo]
#define G 9.81      // �������ٶȣ�m/s^2��
// �������˲����ṹ��
typedef struct {
    float x[N_STATE];       // ״̬����
    float P[N_STATE][N_STATE]; // ״̬Э�������
    float F[N_STATE][N_STATE]; // ״̬ת�ƾ���
    float B[N_STATE][N_CTRL];  // ���ƾ���
    float H[N_OBS][N_STATE];   // �۲����
    float Q[N_STATE][N_STATE]; // ��������Э����
    float R[N_OBS][N_OBS];     // �۲�����Э����
} KalmanFilter;

void Kinematic_Analysis(float Vx,float Vy,float V_angle);
void calculate_odometry(float w1, float w2, float w3, float w4,
                        OdometryState_t * state, float delta_time);
void Base_Control(void *argument);
void Odemetry_Task(void *argument);
void Back_Task(void *arg);

void kalman_init(KalmanFilter* kf, float dt);
void kalman_predict(KalmanFilter* kf, const float* u);
void kalman_update(KalmanFilter* kf, const float* z);
void imu_to_global(float ax_imu, float ay_imu, float theta,
                   float* ax_global, float* ay_global);
// ���������������

#endif //MC_PROJ_BASE_TASK_H

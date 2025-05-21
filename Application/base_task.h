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

// 定义矩阵维度
#define N_STATE 4   // 状态量 [x, y, vx, vy]
#define N_CTRL 2    // 控制量 [ax, ay]
#define N_OBS 2     // 观测量 [x_odo, y_odo]
#define G 9.81      // 重力加速度（m/s^2）
// 卡尔曼滤波器结构体
typedef struct {
    float x[N_STATE];       // 状态向量
    float P[N_STATE][N_STATE]; // 状态协方差矩阵
    float F[N_STATE][N_STATE]; // 状态转移矩阵
    float B[N_STATE][N_CTRL];  // 控制矩阵
    float H[N_OBS][N_STATE];   // 观测矩阵
    float Q[N_STATE][N_STATE]; // 过程噪声协方差
    float R[N_OBS][N_OBS];     // 观测噪声协方差
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
// 矩阵操作函数声明

#endif //MC_PROJ_BASE_TASK_H

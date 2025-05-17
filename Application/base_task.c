//
// Created by 陈瑜 on 25-3-22.
//
#include "all.h"

Base_status_t Base_target_status;//底盘目标状态
OdometryState_t Base_odometry;//底盘里程计

uint8_t mode_flag = SLAVE_MODE;
uint8_t delay_times = 0;
uint8_t delay_flag = 0;
float ax_imu,ay_imu;

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
//            Base_target_status.vx = 20;
//            Base_target_status.vy = 20;
            Kinematic_Analysis(Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
        }
        else if(mode_flag==SLAVE_MODE&&delay_flag==1) {//slave模式，上位机串口控制
            Kinematic_Analysis(Base_target_status.vx, Base_target_status.vy, Base_target_status.omega);
        }

        vTaskDelay(20);
    }
}
/*
  * @brief 里程计任务
  * @author 3YouRan
  * @date 25-5-16 下午5:02
  * @params
  * @return
 */
void Odemetry_Task(void *argument){
    portTickType CurrentTime_PID;
    while(1){//10ms
        CurrentTime_PID=xTaskGetTickCount();
        // 转换IMU加速度到全局坐标系
        float a_global[2];
        imu_to_global(ax_imu, ay_imu, yaw_total, &a_global[0], &a_global[1]);
        // 预测步骤
        kalman_predict(&kf, a_global);
        // 更新步骤
        float z[] = {Base_odometry.x, Base_odometry.y,};
        kalman_update(&kf, z);

            // 输出结果
//            printf("Estimated Position: (%.2f, %.2f)\n",
//                   kf.x[0], kf.x[1]);
        calculate_odometry(motorA.speed,motorB.speed,motorC.speed,motorD.speed,&Base_odometry,0.005f);
        vTaskDelayUntil(&CurrentTime_PID,5);
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

    // 修正后的速度计算（调整了vy的符号）
    state->vx = inv_mat * ( w1 + w2 + w3 + w4) * r;
    state->vy = inv_mat * (-w1 + w2 + w3 - w4) * r; // 修正此行


    // 使用state->theta（弧度）进行积分
    float theta_rad = yaw_total/360.0f*2.0f*M_PI; // 假设theta存储的是弧度
    state->x += (state->vx * cosf(theta_rad) - state->vy * sinf(theta_rad)) * delta_time;
    state->y += (state->vx * sinf(theta_rad) + state->vy * cosf(theta_rad)) * delta_time;



}
KalmanFilter kf;
// 矩阵乘法 (结果 = a * b)
void multiply(float* result, const float* a, int a_rows, int a_cols,
              const float* b, int b_rows, int b_cols) {
    for(int i=0; i<a_rows; i++) {
        for(int j=0; j<b_cols; j++) {
            result[i*b_cols + j] = 0;
            for(int k=0; k<a_cols; k++) {
                result[i*b_cols + j] += a[i*a_cols + k] * b[k*b_cols + j];
            }
        }
    }
}

// 矩阵转置 (结果 = a^T)
void transpose(float* result, const float* a, int rows, int cols) {
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            result[j*rows + i] = a[i*cols + j];
        }
    }
}

// 矩阵加法 (结果 = a + b)
void add(float* result, const float* a, const float* b, int rows, int cols) {
    for(int i=0; i<rows*cols; i++) {
        result[i] = a[i] + b[i];
    }
}

// 2x2矩阵求逆 (结果 = a^(-1))
void inverse2x2(float* result, const float* a) {
    float det = a[0] * a[3] - a[1] * a[2];
    if(fabs(det) < 1e-6) {
        // 奇异矩阵处理
        result[0] = result[1] = result[2] = result[3] = 0;
        return;
    }
    float inv_det = 1.0 / det;
    result[0] =  a[3] * inv_det;
    result[1] = -a[1] * inv_det;
    result[2] = -a[2] * inv_det;
    result[3] =  a[0] * inv_det;
}

// 卡尔曼滤波器初始化
void kalman_init(KalmanFilter* kf, float dt) {
    // 状态转移矩阵 F
    float F[N_STATE][N_STATE] = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    };
    memcpy(kf->F, F, sizeof(F));

    // 控制矩阵 B
    float B[N_STATE][N_CTRL] = {
            {0.5*dt*dt, 0},
            {0, 0.5*dt*dt},
            {dt, 0},
            {0, dt}
    };
    memcpy(kf->B, B, sizeof(B));

    // 观测矩阵 H
    float H[N_OBS][N_STATE] = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
    };
    memcpy(kf->H, H, sizeof(H));

    // 过程噪声协方差 Q (示例值)
    float Q[N_STATE][N_STATE] = {
            {0.9, 0, 0.2, 0},
            {0, 0.9, 0, 0.2},
            {0.2, 0, 0.9, 0},
            {0, 0.2, 0, 0.9}
    };
    memcpy(kf->Q, Q, sizeof(Q));

    // 观测噪声协方差 R (示例值)
    float R[N_OBS][N_OBS] = {
            {0.15, 0.02},   // 考虑X、Y方向的相关性
            {0.02, 0.15}
    };
    memcpy(kf->R, R, sizeof(R));

    // 初始状态和协方差
    for(int i=0; i<N_STATE; i++) {
        kf->x[i] = 0;
        for(int j=0; j<N_STATE; j++) {
            kf->P[i][j] = (i == j) ? 1.0 : 0;
        }
    }
}

// 预测步骤
void kalman_predict(KalmanFilter* kf, const float* u) {
    // 状态预测: x = F*x + B*u
    float Fx[N_STATE];
    multiply(Fx, (float*)kf->F, N_STATE, N_STATE,
             kf->x, N_STATE, 1);

    float Bu[N_STATE];
    multiply(Bu, (float*)kf->B, N_STATE, N_CTRL,
             u, N_CTRL, 1);

    for(int i=0; i<N_STATE; i++) {
        kf->x[i] = Fx[i] + Bu[i];
    }

    // 协方差预测: P = F*P*F^T + Q

    float temp[N_STATE][N_STATE];
    multiply((float*)temp, (float*)kf->F, N_STATE, N_STATE,
             (float*)kf->P, N_STATE, N_STATE); // temp = F * P

    float F_T[N_STATE][N_STATE];
    transpose((float*)F_T, (float*)kf->F, N_STATE, N_STATE); // F^T

    float FPF_T[N_STATE][N_STATE];
    multiply((float*)FPF_T, (float*)temp, N_STATE, N_STATE,
             (float*)F_T, N_STATE, N_STATE); // FPF_T = F * P * F^T

    add((float*)kf->P, (float*)FPF_T, (float*)kf->Q, N_STATE, N_STATE);

}

// 更新步骤
void kalman_update(KalmanFilter* kf, const float* z) {
    // 计算残差 y = z - Hx
    float Hx[N_OBS];
    multiply(Hx, (float*)kf->H, N_OBS, N_STATE,
             kf->x, N_STATE, 1);

    float y[N_OBS];
    for(int i=0; i<N_OBS; i++) {
        y[i] = z[i] - Hx[i];
    }

    // 计算卡尔曼增益 K = P*H^T*(H*P*H^T + R)^-1
    float H_T[N_STATE][N_OBS];
    transpose((float*)H_T, (float*)kf->H, N_OBS, N_STATE);

    float PH_T[N_STATE][N_OBS];
    multiply((float*)PH_T, (float*)kf->P, N_STATE, N_STATE,
             (float*)H_T, N_STATE, N_OBS);

    float HPH_T[N_OBS][N_OBS];
    multiply((float*)HPH_T, (float*)kf->H, N_OBS, N_STATE,
             (float*)PH_T, N_STATE, N_OBS);

    add((float*)HPH_T, (float*)HPH_T, (float*)kf->R,
        N_OBS, N_OBS);

    float inv_HPH_T[N_OBS][N_OBS];
    inverse2x2((float*)inv_HPH_T, (float*)HPH_T);

    float K[N_STATE][N_OBS];
    multiply((float*)K, (float*)PH_T, N_STATE, N_OBS,
             (float*)inv_HPH_T, N_OBS, N_OBS);

    // 更新状态估计
    float Ky[N_STATE];
    multiply(Ky, (float*)K, N_STATE, N_OBS, y, N_OBS, 1);

    for(int i=0; i<N_STATE; i++) {
        kf->x[i] += Ky[i];
    }

    // 更新协方差矩阵
    float KH[N_STATE][N_STATE];
    multiply((float*)KH, (float*)K, N_STATE, N_OBS,
             (float*)kf->H, N_OBS, N_STATE);

    float I[N_STATE][N_STATE] = {0};
    for(int i=0; i<N_STATE; i++) I[i][i] = 1.0;

    float I_KH[N_STATE][N_STATE];
    for(int i=0; i<N_STATE; i++) {
        for(int j=0; j<N_STATE; j++) {
            I_KH[i][j] = I[i][j] - KH[i][j];
        }
    }

    multiply((float*)kf->P, (float*)I_KH, N_STATE, N_STATE,
             (float*)kf->P, N_STATE, N_STATE);
}

// 示例：IMU加速度转全局坐标系
void imu_to_global(float ax_imu, float ay_imu, float theta,
                   float* ax_global, float* ay_global) {
    *ax_global = ax_imu * cos(theta) - ay_imu * sin(theta);
    *ay_global = ax_imu * sin(theta) + ay_imu * cos(theta);
}





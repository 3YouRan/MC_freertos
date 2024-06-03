//
// Created by 陈瑜 on 24-6-3.
//

#ifndef MC_PROJ_KINEMATIC_ANALYSIS_H
#define MC_PROJ_KINEMATIC_ANALYSIS_H

#define RxPLUSRy (0.1097/2+0.1408/2)
/**************************************************************************
麦克纳姆轮逆运动学模型
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle);
#endif //MC_PROJ_KINEMATIC_ANALYSIS_H

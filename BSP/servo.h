//
// Created by 陈瑜 on 25-3-23.
//

#ifndef MC_PROJ_SERVO_H
#define MC_PROJ_SERVO_H

#define SERVO1_SET(Val) __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, Val); //Val(0~1000)
#define SERVO2_SET(Val) __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, Val); //Val(0~1000)
#define SERVO3_SET(Val) __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, Val); //Val(0~1000)
#define PW_MAX 2500 //脉宽2.5ms,对应占空比2.5ms/20ms=12.5%
#define PW_MIN 500 //脉宽0.5ms,对应占空比0.5ms/20ms=2.5%

void servo1_set(float angle);
void servo2_set(float angle);
void servo3_set(float angle);
#endif //MC_PROJ_SERVO_H

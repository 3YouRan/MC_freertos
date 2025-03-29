//
// Created by 陈瑜 on 24-11-1.
//



#include "all.h"
LowPassFilter filter_yaw;
// 初始化滤波器

/**
  * @brief 低通滤波器初始化
  * @author 3YouRan
  *
  * @date 24-11-30 下午7:00
  * @param LowPassFilter* filter 低通滤波器结构体
  * @param float alpha 低通滤波器参数
  *
  * @return
 */
void initialize_LowPassFilter(LowPassFilter* filter, float alpha) {
    filter->alpha = alpha;
    filter->previous_output_1 = 0.0;
}

// 一阶低通滤波函数
/**
  * @brief 一阶低通滤波函数
  * @author 3YouRan
  *
  * @date 24-11-30 下午7:00
  * @param LowPassFilter* filter 低通滤波器结构体
  * @param float input 输入数据
  *
  * @return none
 */
float LowPassFilter_one_step(LowPassFilter* filter, float input) {
    // 计算输出
    float output = (1.0 - filter->alpha) * (filter->previous_output_1) + filter->alpha * input;

    // 更新上一次的输出
    filter->previous_output_1 = output;


    return output;
}



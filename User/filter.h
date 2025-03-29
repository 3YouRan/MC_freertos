//
// Created by 陈瑜 on 24-11-1.
//

#ifndef RC_ALL_ROUND_FILTER_H
#define RC_ALL_ROUND_FILTER_H
typedef struct {
    float alpha;           // 时间常数
    float previous_output_1; // 上一时刻的输出


} LowPassFilter;

void initialize_LowPassFilter(LowPassFilter* filter, float alpha);
float LowPassFilter_one_step(LowPassFilter* filter, float input);

#endif //RC_ALL_ROUND_FILTER_H

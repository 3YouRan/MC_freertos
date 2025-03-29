//
// Created by ��� on 24-11-1.
//

#ifndef RC_ALL_ROUND_FILTER_H
#define RC_ALL_ROUND_FILTER_H
typedef struct {
    float alpha;           // ʱ�䳣��
    float previous_output_1; // ��һʱ�̵����


} LowPassFilter;

void initialize_LowPassFilter(LowPassFilter* filter, float alpha);
float LowPassFilter_one_step(LowPassFilter* filter, float input);

#endif //RC_ALL_ROUND_FILTER_H

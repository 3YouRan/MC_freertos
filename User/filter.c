//
// Created by ��� on 24-11-1.
//



#include "all.h"
LowPassFilter filter_yaw;
// ��ʼ���˲���

/**
  * @brief ��ͨ�˲�����ʼ��
  * @author 3YouRan
  *
  * @date 24-11-30 ����7:00
  * @param LowPassFilter* filter ��ͨ�˲����ṹ��
  * @param float alpha ��ͨ�˲�������
  *
  * @return
 */
void initialize_LowPassFilter(LowPassFilter* filter, float alpha) {
    filter->alpha = alpha;
    filter->previous_output_1 = 0.0;
}

// һ�׵�ͨ�˲�����
/**
  * @brief һ�׵�ͨ�˲�����
  * @author 3YouRan
  *
  * @date 24-11-30 ����7:00
  * @param LowPassFilter* filter ��ͨ�˲����ṹ��
  * @param float input ��������
  *
  * @return none
 */
float LowPassFilter_one_step(LowPassFilter* filter, float input) {
    // �������
    float output = (1.0 - filter->alpha) * (filter->previous_output_1) + filter->alpha * input;

    // ������һ�ε����
    filter->previous_output_1 = output;


    return output;
}



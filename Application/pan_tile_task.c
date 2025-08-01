//
// Created by ��� on 25-3-23.
//

#include <math.h>

#include "all.h"

uint8_t hit_flag=0;
int16_t Pulse_num1=0;
int16_t Pulse_num2=0;
float motor_angle1=0;
float motor_angle2=0;
uint8_t motor_rotate_flag=0;
uint8_t shoot_flag=0;
void pan_tile_task(void *arg) {
    while (1) {
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        if(motor_rotate_flag)
        {
            motor_rotate_flag=0;

            Pulse_num1=Ang_to_Pulse_Num(fabsf(motor_angle1));//�Ƕ�ת������
            Pulse_num2=Ang_to_Pulse_Num(fabsf(motor_angle2));
            if(motor_angle1>0.05) {
                HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
                if(fabsf(motor_angle1)==0.06) {
                    vTaskDelay(100);
                    motor_angle1=0;
                    memset(&DataBuff_UP,0,200); //清零接收缓冲区

                }
                HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1);//����PWM���

            }else if(motor_angle1<-0.05) {
                if(fabsf(motor_angle1)==0.06) {
                    vTaskDelay(100);
                    motor_angle1=0;
                    memset(&DataBuff_UP,0,200); //清零接收缓冲区

                }
                HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
                HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1);//����PWM���

            }
            if(motor_angle2>0.05) {
                if(fabsf(motor_angle2)==0.06) {
                    vTaskDelay(100);
                    motor_angle2=0;
                    memset(&DataBuff_UP,0,200); //清零接收缓冲区

                }
                HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);//����PWM���

            }else if(motor_angle2<-0.05) {
                if(fabsf(motor_angle2)==0.06) {
                    vTaskDelay(100);
                    motor_angle2=0;
                    memset(&DataBuff_UP,0,200); //清零接收缓冲区

                }
                HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);//����PWM���

            }

        }
        if(shoot_flag) {
            HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
            vTaskDelay(200);
            HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
            shoot_flag=0;
        }
        vTaskDelay(5);
    }
}
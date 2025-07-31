//
// Created by ��� on 25-3-22.
//

#include "all.h"

uint8_t Voltage_str[5];
uint8_t yaw_str[7];
uint8_t Space[7]="      ";
int16_t ADC_Value[1];
void f_to_str(float f, uint8_t *str){
    if(f<0){
        str[0]='-';
    }

}

void OLED_Task(void *arg){
//    uint8_t count = 0;
    float voltage;
    while(1){
        //������ת�ַ���
        voltage=ADC_Value[0]/100.0;

        float_to_str(voltage,Voltage_str,2);

        //������ֵ��ʾ��
        OLED_ShowString(65,0,Space);
        OLED_ShowString(32,16,Space);
        OLED_ShowString(48,32,Space);
        //��ʾ��ֵ
        OLED_ShowString(0,0,"Voltage:");//��ʾ��ص�ѹ
        OLED_ShowString(65,0,Voltage_str);


        OLED_Refresh_Gram();//ˢ��
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
        vTaskDelay(500);//��ʱ
    }
}
void float_to_str(float num, char *str, int precision) {
    int integer_part = (int)num;
    float decimal_part = num - integer_part;
    int decimal_int = 0;

    // �����ȣ����磬precision=2ʱ����ȡ��λС����
    for (int i = 0; i < precision; i++) {
        decimal_part *= 10;
    }
    decimal_int = (int)decimal_part;
    if (decimal_int <0){
        decimal_int = -decimal_int;
    }
    // ����ַ���
    sprintf(str, "%d.%0*d", integer_part, precision, decimal_int);
}
//
// Created by 陈瑜 on 25-3-22.
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
        //浮点数转字符串
        voltage=ADC_Value[0]/100.0;

        float_to_str(voltage,Voltage_str,2);

        //重置数值显示区
        OLED_ShowString(65,0,Space);
        OLED_ShowString(32,16,Space);
        OLED_ShowString(48,32,Space);
        //显示数值
        OLED_ShowString(0,0,"Voltage:");//显示电池电压
        OLED_ShowString(65,0,Voltage_str);


        OLED_Refresh_Gram();//刷新
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
        vTaskDelay(500);//延时
    }
}
void float_to_str(float num, char *str, int precision) {
    int integer_part = (int)num;
    float decimal_part = num - integer_part;
    int decimal_int = 0;

    // 处理精度（例如，precision=2时，提取两位小数）
    for (int i = 0; i < precision; i++) {
        decimal_part *= 10;
    }
    decimal_int = (int)decimal_part;
    if (decimal_int <0){
        decimal_int = -decimal_int;
    }
    // 组合字符串
    sprintf(str, "%d.%0*d", integer_part, precision, decimal_int);
}
//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"
uint8_t V_A[5];
uint8_t V_B[5];
uint8_t V_C[5];
uint8_t V_D[5];
uint8_t Voltage_str[5];

uint8_t Space[5]="    ";
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
        voltage=ADC_Value[0]/4096.0*5*11;
        sprintf(V_A,"%d",(int16_t )motorA.speed);
        sprintf(V_B,"%d",-(int16_t )motorB.speed);
        sprintf(V_C,"%d",(int16_t )motorC.speed);
        sprintf(V_D,"%d",-(int16_t )motorD.speed);
        float_to_str(voltage,Voltage_str,2);
        //重置数值显示区
        OLED_ShowString(65,0,Space);
        OLED_ShowString(24,16,Space);
        OLED_ShowString(24,32,Space);
        OLED_ShowString(88,16,Space);
        OLED_ShowString(88,32,Space);

        //显示数值
        OLED_ShowString(0,0,"Voltage:");//显示电池电压
        OLED_ShowString(65,0,Voltage_str);

        OLED_ShowString(0,16,"VA:");//显示电机A速度
        OLED_ShowString(24,16,V_A);


        OLED_ShowString(0,32,"VC:");//显示电机B速度
        OLED_ShowString(24,32,V_C);

        OLED_ShowString(64,16,"VB:");//显示电机C速度
        OLED_ShowString(88,16,V_B);

        OLED_ShowString(64,32,"VD:");//显示电机D速度
        OLED_ShowString(88,32,V_D);


        OLED_Refresh_Gram();//刷新
        vTaskDelay(250);//延时
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

    // 组合字符串
    sprintf(str, "%d.%0*d", integer_part, precision, decimal_int);
}
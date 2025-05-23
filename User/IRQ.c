//
// Created by 陈瑜 on 25-3-22.
//

#include "all.h"

uint16_t RxLine = 0;//指令长度
uint8_t RxBuffer[1];//串口接收缓冲
uint8_t DataBuff[200];//指令内容
uint8_t UART_num = 0;
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        UART_num=2;
        RxLine++;                            // 接收行数加1
        DataBuff[RxLine-1]=RxBuffer[0];       // 将接收到的数据存入缓冲区
        if(RxBuffer[0]=='!')                  // 判断是否接收到感叹号
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
            USART_PID_Adjust(1,DataBuff);             // 调整USART PID


            memset(DataBuff,0,sizeof(DataBuff));   // 清空接收缓冲区
            RxLine=0;                           // 接收行数清零
        }
        RxBuffer[0]=0;                        // 重置接收缓冲区

        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, 1);   // 重新启动UART接收中断
    }

}

uint8_t debugRvAll[DEBUG_RV_MXSIZE] = {0};
void Set_Target_UartInit()
{
    //  数据接收
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//使能串口6的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataBuff_UP, 200);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据

}

void DMA_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)//判断是否是串口6
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断

            DMA_Imu600_UartIdleCallback(huart);//调用中断处理函数

        }
    }
    if(huart->Instance == huart3.Instance)//判断是否是串口2
    {

        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断

            DMA_UP_UartIdleCallback(huart);//调用中断处理函数

        }
    }
}
void imu600_parse(uint8_t *buf){
    U16 ctl;
    U8 L;
    U8 tmpU8;
    U16 tmpU16;
    U32 tmpU32;
    F32 tmpX, tmpY, tmpZ, tmpAbs;
    if (buf[0]== 0x11) // 获取订阅的功能数据 回复或主动上报
    {
            ctl = ((U16)buf[2] << 8) | buf[1];// 字节[2-1] 为功能订阅标识，指示当前订阅了哪些功能
    //            Dbp("\t subscribe tag: 0x%04X\r\n", ctl);
    //            Dbp("\t ms: %u\r\n", (U32)(((U32)buf[6]<<24) | ((U32)buf[5]<<16) | ((U32)buf[4]<<8) | ((U32)buf[3]<<0))); // 字节[6-3] 为模块开机后的时间戳(单位ms)

        L =7; // 从第7字节开始根据 订阅标识tag来解析剩下的数据
        if ((ctl & 0x0001) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2;// Dbp("\taX: %.3f\r\n", tmpX); // x加速度aX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2;// Dbp("\taY: %.3f\r\n", tmpY); // y加速度aY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\taZ: %.3f\r\n", tmpZ); // z加速度aZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\ta_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
            ax_imu=tmpX;
            ay_imu=tmpY;
        }
        if ((ctl & 0x0002) != 0)
        {// 加速度xyz 包含了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAX: %.3f\r\n", tmpX); // x加速度AX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAY: %.3f\r\n", tmpY); // y加速度AY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAZ: %.3f\r\n", tmpZ); // z加速度AZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tA_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0004) != 0)
        {// 角速度xyz 使用时需*scaleAngleSpeed °/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGX: %.3f\r\n", tmpX); // x角速度GX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGY: %.3f\r\n", tmpY); // y角速度GY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGZ: %.3f\r\n", tmpZ); // z角速度GZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tG_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0008) != 0)
        {// 磁场xyz 使用时需*scaleMag uT
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCX: %.3f\r\n", tmpX); // x磁场CX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCY: %.3f\r\n", tmpY); // y磁场CY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCZ: %.3f\r\n", tmpZ); // z磁场CZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tC_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0010) != 0)
        {// 温度 气压 高度
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; Dbp("\ttemperature: %.2f\r\n", tmpX); // 温度

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpY = (S32)tmpU32 * scaleAirPressure; L += 3; Dbp("\tairPressure: %.3f\r\n", tmpY); // 气压

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpZ = (S32)tmpU32 * scaleHeight; L += 3; Dbp("\theight: %.3f\r\n", tmpZ); // 高度
        }
        if ((ctl & 0x0020) != 0)
        {// 四元素 wxyz 使用时需*scaleQuat
            tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tw: %.3f\r\n", tmpAbs); // w
            tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tx: %.3f\r\n", tmpX); // x
            tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\ty: %.3f\r\n", tmpY); // y
            tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tz: %.3f\r\n", tmpZ); // z
        }
        if ((ctl & 0x0040) != 0)
        {// 欧拉角xyz 使用时需*scaleAngle
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleX: %.3f\r\n", tmpX); // x角度
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleY: %.3f\r\n", tmpY); // y角度
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleZ: %.3f\r\n", tmpZ); // z角度
            yaw_last = yaw;
            yaw=tmpZ;
            if(yaw-yaw_last>180){
                yaw_total+=yaw-yaw_last-360;
            }else if(yaw-yaw_last<-180){
                yaw_total+=yaw-yaw_last+360;
            }else{
                yaw_total+=yaw-yaw_last;
            }
//            usart_printf("%.2f,%.2f,%.2f!",tmpX,tmpY,tmpZ);
        }
        if ((ctl & 0x0080) != 0)
        {// xyz 空间位移 单位mm 转为 m
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetX: %.3f\r\n", tmpX); // x坐标
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetY: %.3f\r\n", tmpY); // y坐标
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetZ: %.3f\r\n", tmpZ); // z坐标
        }
        if ((ctl & 0x0100) != 0)
        {// 活动检测数据
            tmpU32 = (U32)(((U32)buf[L+3]<<24) | ((U32)buf[L+2]<<16) | ((U32)buf[L+1]<<8) | ((U32)buf[L]<<0)); L += 4; Dbp("\tsteps: %u\r\n", tmpU32); // 计步数
            tmpU8 = buf[L]; L += 1;
            Dbp("\t walking: %s\r\n", (tmpU8 & 0x01)?  "yes" : "no"); // 是否在走路
            Dbp("\t running: %s\r\n", (tmpU8 & 0x02)?  "yes" : "no"); // 是否在跑步
            Dbp("\t biking: %s\r\n",  (tmpU8 & 0x04)?  "yes" : "no"); // 是否在骑车
            Dbp("\t driving: %s\r\n", (tmpU8 & 0x08)?  "yes" : "no"); // 是否在开车
        }
        if ((ctl & 0x0200) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasX: %.3f\r\n", tmpX); // x加速度asX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasY: %.3f\r\n", tmpY); // y加速度asY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tasZ: %.3f\r\n", tmpZ); // z加速度asZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\tas_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值


        }
        if ((ctl & 0x0400) != 0)
        {// ADC的值
            tmpU16 = (U16)(((U16)buf[L+1]<<8) | ((U16)buf[L]<<0)); L += 2; Dbp("\tadc: %u\r\n", tmpU16); // 单位mv
        }
        if ((ctl & 0x0800) != 0)
        {// GPIO1的值
            tmpU8 = buf[L]; L += 1;
            Dbp("\t GPIO1  M:%X, N:%X\r\n", (tmpU8>>4)&0x0f, (tmpU8)&0x0f);
        }
    }
}

void DMA_Imu600_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //将接收到的数据存入数组
//    printf("UART6:%x,%x,%x,%x,%x\r\n",debugRvAll[0],debugRvAll[1],debugRvAll[2],debugRvAll[3],debugRvAll[4]);
    imu600_parse(&debugRvAll[3]);
    memset(&debugRvAll,0,data_length); //清零接收缓冲区

    HAL_UART_Receive_DMA(huart, (uint8_t*)&debugRvAll, DEBUG_RV_MXSIZE);//循环中开启串口的DMA接收

}
void DMA_UP_UartIdleCallback(UART_HandleTypeDef *huart)//注意一个问题，调用的时候再写&huart6，否则在这个函数里会出问题
{
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    //将接收到的数据存入数组
//    printf("UART6:%x,%x,%x,%x,%x\r\n",debugRvAll[0],debugRvAll[1],debugRvAll[2],debugRvAll[3],debugRvAll[4]);
    USART_PID_Adjust(1,DataBuff_UP);
    memset(&DataBuff_UP,0,data_length); //清零接收缓冲区

    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DataBuff_UP, 200);   // 启动UART接收中断

}


int times1=0;
short encoder_now1=0;
short encoder_now2=0;
short encoder_now3=0;
short encoder_now4=0;

float Target_Speed_Inc=3;
float Target_Angle_Inc=1.5f;

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    //calculate_odometry(motorA.speed,motorB.speed,motorC.speed,motorD.speed,&Base_odometry,0.002f);
    if (htim==&htim6){// 2ms
        times1++;
        encoder_now1=(short )(__HAL_TIM_GET_COUNTER(ENCODER1));
        encoder_now2=(short )(__HAL_TIM_GET_COUNTER(ENCODER2));
        encoder_now3=(short )(__HAL_TIM_GET_COUNTER(ENCODER3));
        encoder_now4=(short )(__HAL_TIM_GET_COUNTER(ENCODER4));
        motorA.totalCount+=encoder_now1;
        motorB.totalCount+=encoder_now2;
        motorC.totalCount+=encoder_now3;
        motorD.totalCount+=encoder_now4;
        __HAL_TIM_SET_COUNTER(ENCODER1,0);
        __HAL_TIM_SET_COUNTER(ENCODER2,0);
        __HAL_TIM_SET_COUNTER(ENCODER3,0);
        __HAL_TIM_SET_COUNTER(ENCODER4,0);
        motorA.speed=(float )encoder_now1/(4*28*500)*500*60;
        motorB.speed=-(float )encoder_now2/(4*28*500)*500*60; //rpm
        motorC.speed=(float )encoder_now3/(4*28*500)*500*60;
        motorD.speed=-(float )encoder_now4/(4*30*500)*500*60;

        if(times1==10){//20ms
            times1=0;

            //目标速度爬坡
            if (motorA.TargetSpeed_now<motorA.TargetSpeed){
                motorA.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorA.TargetSpeed_now>motorA.TargetSpeed){
                motorA.TargetSpeed_now-=Target_Speed_Inc;
            }else if(fabsf(motorA.TargetSpeed_now-motorA.TargetSpeed)<Target_Speed_Inc){
                motorA.TargetSpeed_now=motorA.TargetSpeed;
            }
            if (motorB.TargetSpeed_now<motorB.TargetSpeed){
                motorB.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorB.TargetSpeed_now>motorB.TargetSpeed){
                motorB.TargetSpeed_now-=Target_Speed_Inc;
            } else if(fabsf(motorB.TargetSpeed_now-motorB.TargetSpeed)<Target_Speed_Inc){
                motorB.TargetSpeed_now=motorB.TargetSpeed;
            }
            if (motorC.TargetSpeed_now<motorC.TargetSpeed){
                motorC.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorC.TargetSpeed_now>motorC.TargetSpeed){
                motorC.TargetSpeed_now-=Target_Speed_Inc;
            }else if(fabsf(motorC.TargetSpeed_now-motorC.TargetSpeed)<Target_Speed_Inc){
                motorC.TargetSpeed_now=motorC.TargetSpeed;
            }
            if (motorD.TargetSpeed_now<motorD.TargetSpeed){
                motorD.TargetSpeed_now+=Target_Speed_Inc;
            }else if(motorD.TargetSpeed_now>motorD.TargetSpeed){
                motorD.TargetSpeed_now-=Target_Speed_Inc;
            } else if(fabsf(motorD.TargetSpeed_now-motorD.TargetSpeed)<Target_Speed_Inc){
                motorD.TargetSpeed_now=motorD.TargetSpeed;
            }
            //目标角度爬坡
            if(Target_Angle_actual-Target_Angle>Target_Angle_Inc){
                Target_Angle_actual-=Target_Angle_Inc;
            }else if(Target_Angle_actual-Target_Angle<-Target_Angle_Inc){
                Target_Angle_actual+=Target_Angle_Inc;
            }else if(fabsf(Target_Angle_actual-Target_Angle)<Target_Angle_Inc){
                Target_Angle_actual=Target_Angle;
            }
        }
    }
//    if (htim->Instance == TIM12) {//1ms
//
//
//    }
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }

    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}
#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];

void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);

    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);

    va_end(args);

    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)send_buf, length);
}
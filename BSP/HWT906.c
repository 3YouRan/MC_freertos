//
// Created by 陈瑜 on 25-3-24.
//


#include "all.h"

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV    stcGPSV;
struct SQ       stcQ;
unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;
void CopeSerial2Data(unsigned char ucData)
{


    ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<11)
    {
        return;   //数据不满11个，则返回
    }
    else
    {
        switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
        {
            case 0x50:
                memcpy(&stcTime,&ucRxBuffer[2],8);
                break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
            case 0x51:
                memcpy(&stcAcc,&ucRxBuffer[2],8);
                break;
            case 0x52:
                memcpy(&stcGyro,&ucRxBuffer[2],8);
                break;
            case 0x53:
                memcpy(&stcAngle,&ucRxBuffer[2],8);
                break;
            case 0x54:
                memcpy(&stcMag,&ucRxBuffer[2],8);
                break;
            case 0x55:
                memcpy(&stcDStatus,&ucRxBuffer[2],8);
                break;
            case 0x56:
                memcpy(&stcPress,&ucRxBuffer[2],8);
                break;
            case 0x57:
                memcpy(&stcLonLat,&ucRxBuffer[2],8);
                break;
            case 0x58:
                memcpy(&stcGPSV,&ucRxBuffer[2],8);
                break;
            case 0x59:
                memcpy(&stcQ,&ucRxBuffer[2],8);
                break;
        }
        ucRxCnt=0;//清空缓存区
    }
}
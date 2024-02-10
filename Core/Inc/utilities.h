//
// Created by mulai on 2024/2/3.
//

#ifndef _UTILITIES_H
#define _UTILITIES_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "stdio.h"

typedef float fp32;

typedef struct {
    fp32 input;         //输入数据
    fp32 out;           //滤波输出的数据
    fp32 num;           //滤波参数
    fp32 frame_period;  //滤波的时间间隔 单位 s
} first_order_filter_type_t;
// 定义低通滤波器结构体
typedef struct {
    float alpha;           // 时间常数
    float previous_output; // 上一时刻的输出
} LowPassFilter;

typedef struct
{
    float *pBuf;
    uint8_t ItemNum;
    uint8_t ItemCnt;
    uint8_t ItemIdx;
    float fSum;
    float out;
}RollingMeanFilterValue;

typedef struct
{
    float Last_P;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
}Kalman;

void Printf(char *format, ...);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num);
void first_order_filter_calc(first_order_filter_type_t *first_order_filter_type, fp32 input);
// 初始化滤波器
void LowPassFilter_Init(LowPassFilter* filter, float alpha);
float LowPassFilter_Calc(LowPassFilter* filter, float input);

void RollingMeanFilter_Init(RollingMeanFilterValue *Filter, float *pBuf, uint8_t ItemNum);
float RollingMeanFilter(RollingMeanFilterValue *Filter, float input);

void Kalman_Init(Kalman *kfp, float Q, float R);
float KalmanFilter(Kalman *kfp,float input);

#ifdef __cplusplus
}
#endif
#endif

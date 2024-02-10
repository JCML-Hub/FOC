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
    fp32 input;         //��������
    fp32 out;           //�˲����������
    fp32 num;           //�˲�����
    fp32 frame_period;  //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
// �����ͨ�˲����ṹ��
typedef struct {
    float alpha;           // ʱ�䳣��
    float previous_output; // ��һʱ�̵����
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
// ��ʼ���˲���
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

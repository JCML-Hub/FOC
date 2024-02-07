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

void Printf(char *format, ...);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num);
void first_order_filter_calc(first_order_filter_type_t *first_order_filter_type, fp32 input);
#ifdef __cplusplus
}
#endif
#endif

/*
 * @Author       : LuHeQiu
 * @Date         : 2022-01-12 22:33:07
 * @LastEditTime : 2022-12-04 18:19:14
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : \motor-controller-with-foc\Software\MainController\Application\function.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __FUNCTION_H
#define __FUNCTION_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "usart.h"

typedef  int32_t Fun32BitsNumber;
typedef  float fp32;

#define Abs(value)                 (((value)>=0)?(value):(0-(value)))
#define Constrain(input,low,high)  ((input)<(low)?(low):((input)>(high)?(high):(input))) 
#define Max(A,B)                   ((A)>=(B)?(A):(B))
#define Min(A,B)                   ((A)<=(B)?(A):(B))
//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

#define PI      3.14159265f
#define _2PI    6.28318530f

enum PID_MODE { PID_POSITION = 0, PID_DELTA };

typedef struct pid_type {
    uint8_t mode;
    // PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;   //最大输出
    fp32 max_iout;  //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];   //微分项 0最新 1上一次 2上上次
    fp32 error[3];  //误差项 0最新 1上一次 2上上次

} pid_type_t;

void pid_init(pid_type_t *pid, uint8_t mode, const fp32 PID[5]);
fp32 pid_calc(pid_type_t *pid, fp32 ref, fp32 set);
void pid_clear(pid_type_t *pid);
void pid_clear_i(pid_type_t *pid);

/**
* @brief  atoi ( ascii to integer) 为把字符串转换成整型数的一个函数
* @param  nptr 字符串指针
* @retval 被转换完成的整型数
*/
int atoi(const char *nptr);

/**
* @brief  itoa ( integer to ascii) 为把整型数转换为字符串的一个函数
* @param  value 待转换的
* @param  string 存储转换结果的指针，未作溢出处理，请自己确保开辟的空间足够
* @param  radix 转换的进制，例如 10 为按照10进制转换。
* @retval 转换成的字符串(不推荐使用)
*/
char *itoa(int value, char *string, int radix);

/**
 * @brief  基于查找表(look-up table)的sin函数，使用一个大小为256的表扩展映射四倍用于sin函数查找。
 *         (sin/cos共用同一张容量为256个float的表)
 * @param  theta 弧度制角度
 * @retval sin(theta)
 */
float SinByLut(float theta);

/**
 * @brief  基于查找表(look-up table)的cos函数，使用一个大小为256的表扩展映射四倍用于cos函数查找。
 *         (sin/cos共用同一张容量为256个float的表)
 * @param  theta 弧度制角度
 * @retval cos(theta)
 */
float CosByLut(float theta);


float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

#ifdef __cplusplus
}
#endif

#endif

//
// Created by mulai on 2024/2/3.
//

#include "utilities.h"

#include "usart.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);
  return ch;
}


void Printf(char *format, ...){//重定义一个printf
  uint8_t UserTxBufferFS[128];
  uint32_t length;

  length = sprintf((char *)UserTxBufferFS, format);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UserTxBufferFS, length);
  huart1.gState = HAL_UART_STATE_READY;
}

/**
 * @brief          一阶低通滤波初始化
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num          = num;
  first_order_filter_type->input        = 0.0f;
  first_order_filter_type->out          = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void first_order_filter_calc(first_order_filter_type_t *first_order_filter_type, fp32 input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
          first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
          first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

// 初始化滤波器
void LowPassFilter_Init(LowPassFilter* filter, float alpha) {
  filter->alpha = alpha;
  filter->previous_output = 0.0f;
}
// 低通滤波函数
float LowPassFilter_Calc(LowPassFilter* filter, float input) {
  // 计算输出
  float output = (float )(1.0 - filter->alpha) * filter->previous_output
          + filter->alpha * input;
  // 更新上一次的输出
  filter->previous_output = output;
  return output;
}

//滚动均值滤波  可将其视为一个队列 将来一个出去一个 在达到平衡时取其均值
void RollingMeanFilter_Init(RollingMeanFilterValue *Filter, float *pBuf, uint8_t ItemNum)
{
  Filter->pBuf = pBuf;
  Filter->ItemNum = ItemNum;
  Filter->ItemCnt = 0;
  Filter->ItemIdx = 0;
  Filter->fSum = 0;
}

float RollingMeanFilter(RollingMeanFilterValue *Filter, float input)
{
  if (Filter->ItemCnt < Filter->ItemNum)
  {//积累原始值
    Filter->ItemCnt++;
  }
  Filter->fSum -=Filter->pBuf[Filter->ItemIdx];//排出数据尾部数据
  Filter->pBuf[Filter->ItemIdx] = input;
  Filter->fSum +=input;//加入新数据
  Filter->ItemIdx++;
  if (Filter->ItemIdx >= Filter->ItemNum)
  {//特殊情况排除
    Filter->ItemIdx=0;
  }
  Filter->out = Filter->fSum / (float )Filter->ItemCnt;//求均值
  return Filter->out;
}

void Kalman_Init(Kalman *kfp, float Q, float R)
{
  kfp->Last_P = 1;
  kfp->Now_P = 0;
  kfp->out = 0;
  kfp->Kg = 0.1f;
  kfp->Q = Q;
  kfp->R = R;//该值对旋转编码器的采样值
}
/*
	Q值为过程噪声，越小系统越容易收敛，我们对模型预测的值信任度越高；
	但是太小则容易发散，如果Q为零，那么我们只相信预测值；
	Q值越大我们对于预测的信任度就越低，而对测量值的信任度就变高；
	如果Q值无穷大，那么我们只信任测量值；

	R值为测量噪声，太小太大都不一定合适。
	R太大，卡尔曼滤波响应会变慢，因为它对新测量的值的信任度降低；
	越小系统收敛越快，但过小则容易出现震荡；
	R值的改变主要是影响卡尔曼的收敛速度。

	测试时可以先将Q从小往大调整，将R从大往小调整；
	先固定一个值去调整另外一个值，看收敛速度与波形输出。

	系统中还有一个关键值P，它是误差协方差初始值，
	表示我们对当前预测状态的信任度，它越小说明我们越相信当前预测状态；
	它的值决定了初始收敛速度，一般开始设一个较小的值以便于获取较快的收敛速度。
	随着卡尔曼滤波的迭代，P的值会不断的改变，当系统进入稳态之后P值会收敛成一个最小的估计方差矩阵，这
	个时候的卡尔曼增益也是最优的，所以这个值只是影响初始收敛速度。
*/

/**
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman *kfp,float input)
{
  //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
  kfp->Now_P = kfp->Last_P + kfp->Q;
  //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
  kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
  //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
  kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
  //更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
  kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
  return kfp->out;
}


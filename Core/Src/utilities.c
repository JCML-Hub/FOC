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


void Printf(char *format, ...){//�ض���һ��printf
  uint8_t UserTxBufferFS[128];
  uint32_t length;

  length = sprintf((char *)UserTxBufferFS, format);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UserTxBufferFS, length);
  huart1.gState = HAL_UART_STATE_READY;
}

/**
 * @brief          һ�׵�ͨ�˲���ʼ��
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      �˲�����
 * @retval         ���ؿ�
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num          = num;
  first_order_filter_type->input        = 0.0f;
  first_order_filter_type->out          = 0.0f;
}

/**
 * @brief          һ�׵�ͨ�˲�����
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @retval         ���ؿ�
 */
void first_order_filter_calc(first_order_filter_type_t *first_order_filter_type, fp32 input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
          first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
          first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

// ��ʼ���˲���
void LowPassFilter_Init(LowPassFilter* filter, float alpha) {
  filter->alpha = alpha;
  filter->previous_output = 0.0f;
}
// ��ͨ�˲�����
float LowPassFilter_Calc(LowPassFilter* filter, float input) {
  // �������
  float output = (float )(1.0 - filter->alpha) * filter->previous_output
          + filter->alpha * input;
  // ������һ�ε����
  filter->previous_output = output;
  return output;
}

//������ֵ�˲�  �ɽ�����Ϊһ������ ����һ����ȥһ�� �ڴﵽƽ��ʱȡ���ֵ
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
  {//����ԭʼֵ
    Filter->ItemCnt++;
  }
  Filter->fSum -=Filter->pBuf[Filter->ItemIdx];//�ų�����β������
  Filter->pBuf[Filter->ItemIdx] = input;
  Filter->fSum +=input;//����������
  Filter->ItemIdx++;
  if (Filter->ItemIdx >= Filter->ItemNum)
  {//��������ų�
    Filter->ItemIdx=0;
  }
  Filter->out = Filter->fSum / (float )Filter->ItemCnt;//���ֵ
  return Filter->out;
}

void Kalman_Init(Kalman *kfp, float Q, float R)
{
  kfp->Last_P = 1;
  kfp->Now_P = 0;
  kfp->out = 0;
  kfp->Kg = 0.1f;
  kfp->Q = Q;
  kfp->R = R;//��ֵ����ת�������Ĳ���ֵ
}
/*
	QֵΪ����������ԽСϵͳԽ�������������Ƕ�ģ��Ԥ���ֵ���ζ�Խ�ߣ�
	����̫С�����׷�ɢ�����QΪ�㣬��ô����ֻ����Ԥ��ֵ��
	QֵԽ�����Ƕ���Ԥ������ζȾ�Խ�ͣ����Բ���ֵ�����ζȾͱ�ߣ�
	���Qֵ�������ô����ֻ���β���ֵ��

	RֵΪ����������̫С̫�󶼲�һ�����ʡ�
	R̫�󣬿������˲���Ӧ���������Ϊ�����²�����ֵ�����ζȽ��ͣ�
	ԽСϵͳ����Խ�죬����С�����׳����𵴣�
	Rֵ�ĸı���Ҫ��Ӱ�쿨�����������ٶȡ�

	����ʱ�����Ƚ�Q��С�����������R�Ӵ���С������
	�ȹ̶�һ��ֵȥ��������һ��ֵ���������ٶ��벨�������

	ϵͳ�л���һ���ؼ�ֵP���������Э�����ʼֵ��
	��ʾ���ǶԵ�ǰԤ��״̬�����ζȣ���ԽС˵������Խ���ŵ�ǰԤ��״̬��
	����ֵ�����˳�ʼ�����ٶȣ�һ�㿪ʼ��һ����С��ֵ�Ա��ڻ�ȡ�Ͽ�������ٶȡ�
	���ſ������˲��ĵ�����P��ֵ�᲻�ϵĸı䣬��ϵͳ������̬֮��Pֵ��������һ����С�Ĺ��Ʒ��������
	��ʱ��Ŀ���������Ҳ�����ŵģ��������ֵֻ��Ӱ���ʼ�����ٶȡ�
*/

/**
 *�������˲���
 *@param 	Kalman *kfp �������ṹ�����
 *   			float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float KalmanFilter(Kalman *kfp,float input)
{
  //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
  kfp->Now_P = kfp->Last_P + kfp->Q;
  //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
  kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
  //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
  kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
  //����Э�����: ���ε�ϵͳЭ����� kfp->LastP Ϊ��һ������׼����
  kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
  return kfp->out;
}


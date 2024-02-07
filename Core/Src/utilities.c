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


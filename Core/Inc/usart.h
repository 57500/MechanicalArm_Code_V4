/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE1 64 // 定义缓冲区大小，足够存下一行数据即可


  typedef struct
  {
    uint8_t rx_byte;           // 临时接收 1 个字节
    char rx_buffer[RX_BUFFER_SIZE1]; // 接收缓冲区字符串
    uint8_t rx_index;      // 缓冲区索引
    uint8_t parse_flag;    // 解析标志位
  }UART1_t;

  extern UART1_t uart1;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART_SendFloat_sprintf(UART_HandleTypeDef *huart, float value, uint8_t decimals);

void Send_FireWater_Text(float* data, int num_channels);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


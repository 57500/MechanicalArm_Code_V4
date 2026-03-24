/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "stdbool.h"
#include "usart.h"
const float M_PI = 3.14159265358979323846f;

float dh_theta[6]={0,-M_PI/2,M_PI/2,0,-M_PI/2,0};
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 14;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// bool MyCAN1_FilterInit(void)
// {
//   CAN_FilterTypeDef MyCAN1_Filter;
//   uint32_t start_tick = HAL_GetTick();
//
//   MyCAN1_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
//   MyCAN1_Filter.FilterActivation=ENABLE;
//   MyCAN1_Filter.FilterBank=0;
//   MyCAN1_Filter.FilterFIFOAssignment=CAN_RX_FIFO0;
//   MyCAN1_Filter.FilterIdHigh=0x0;
//   MyCAN1_Filter.FilterIdLow=0x0;
//   MyCAN1_Filter.FilterMaskIdHigh=0x0;
//   MyCAN1_Filter.FilterMaskIdLow=0x0;
//   MyCAN1_Filter.SlaveStartFilterBank=14;
//   MyCAN1_Filter.FilterScale=CAN_FILTERSCALE_32BIT;
//
//   do
//   {
//     if (HAL_CAN_ConfigFilter(&hcan1,&MyCAN1_Filter) == HAL_OK)
//     {
//       return true;
//     }
//   }while (HAL_GetTick()-start_tick<1000);
//
//   return false;
// }

bool MyCAN1_FilterInit(void)
{
  CAN_FilterTypeDef sFilterConfig;

  // --- 过滤器 0：匹配电机 1, 2, 3 -> FIFO 0 ---
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // 使用列表模式，更精准
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // 电机 1 和 2 的 ID (假设 packnum 为 0)
  uint32_t id1 = (0x01 << 8) << 3 | CAN_ID_EXT;
  uint32_t id2 = (0x02 << 8) << 3 | CAN_ID_EXT;
  sFilterConfig.FilterIdHigh = (uint16_t)(id1 >> 16);
  sFilterConfig.FilterIdLow = (uint16_t)(id1 & 0xFFFF);
  sFilterConfig.FilterMaskIdHigh = (uint16_t)(id2 >> 16);
  sFilterConfig.FilterMaskIdLow = (uint16_t)(id2 & 0xFFFF);

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // --- 过滤器 1：匹配电机 3 -> FIFO 0 (继续占坑) ---
  sFilterConfig.FilterBank = 1;
  uint32_t id3 = (0x03 << 8) << 3 | CAN_ID_EXT;
  sFilterConfig.FilterIdHigh = (uint16_t)(id3 >> 16);
  sFilterConfig.FilterIdLow = (uint16_t)(id3 & 0xFFFF);
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // --- 过滤器 2：匹配电机 4, 5 -> FIFO 1 ---
  sFilterConfig.FilterBank = 2;
  uint32_t id4 = (0x04 << 8) << 3 | CAN_ID_EXT;
  uint32_t id5 = (0x05 << 8) << 3 | CAN_ID_EXT;
  sFilterConfig.FilterIdHigh = (uint16_t)(id4 >> 16);
  sFilterConfig.FilterIdLow = (uint16_t)(id4 & 0xFFFF);
  sFilterConfig.FilterMaskIdHigh = (uint16_t)(id5 >> 16);
  sFilterConfig.FilterMaskIdLow = (uint16_t)(id5 & 0xFFFF);
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // 指向 FIFO 1
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // --- 过滤器 3：匹配电机 6 -> FIFO 1 ---
  sFilterConfig.FilterBank = 3;
  uint32_t id6 = (0x06 << 8) << 3 | CAN_ID_EXT;
  sFilterConfig.FilterIdHigh = (uint16_t)(id6 >> 16);
  sFilterConfig.FilterIdLow = (uint16_t)(id6 & 0xFFFF);
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // 开启中断通知
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

  return true;
}


bool MyCAN1_Transmit(const uint8_t* Data, const uint8_t Length)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t packnum = 0;
  uint8_t pos = 0;
  uint32_t start_tick = HAL_GetTick();

  // 地址是Data[0]，功能码是Data[1]
  uint8_t address = Data[0];
  uint8_t func_code = Data[1];
  uint8_t data_length = Length - 2;  // 除去地址和功能码

  while (pos < data_length)
  {
    uint8_t remaining = data_length - pos;

    TxHeader.StdId = 0x00;
    TxHeader.ExtId = (address << 8) | packnum;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint8_t TxData[8] = {0};

    // 第一字节固定为功能码
    TxData[0] = func_code;

    if (remaining < 7)
    {
      // 剩余数据+功能码 <= 8字节
      TxHeader.DLC = remaining + 1;

      for (uint8_t i = 0; i < remaining; i++)
      {
        TxData[i + 1] = Data[pos + 2];  // +2跳过地址和功能码
        pos++;
      }
    }
    else
    {
      // 发送完整8字节（功能码 + 7个数据字节）
      TxHeader.DLC = 8;

      for (uint8_t i = 0; i < 7; i++)
      {
        TxData[i + 1] = Data[pos + 2];
        pos++;
      }
    }

    while (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      if (HAL_GetTick() - start_tick > 10)
      {
        return false;
      }
    }

    packnum++;
  }


  return true;

}

CAN1_t can1={0};

// 提取出的解析逻辑
void CAN_Process_Motor_Data(CAN_RxHeaderTypeDef *pHeader, uint8_t *pData, uint8_t fifo_num) {
  uint8_t motor_id = pHeader->ExtId >> 8;
  uint8_t pack_num = pHeader->ExtId & 0xFF;

  // 安全检查
  if (motor_id < 1 || motor_id > 6) return;
  uint8_t idx = motor_id - 1;

  if (pack_num == 0) {
    // 存入对应电机的独立抽屉
    can1.rx_buf[idx][0] = motor_id;
    for (uint8_t i = 0; i < pHeader->DLC; i++) {
      can1.rx_buf[idx][i + 1] = pData[i];
    }

    // 解析单包或第一包
    if (pHeader->DLC < 8) {
      if (can1.rx_buf[idx][1] == 0x36) { // 功能码
        can1.rx_flag[idx] = 1;

        // 修复：位移前强制转为 uint32_t，防止 8位转32位时的符号位扩展错误
        uint32_t Pos = ((uint32_t)can1.rx_buf[idx][3] << 24) |
                       ((uint32_t)can1.rx_buf[idx][4] << 16) |
                       ((uint32_t)can1.rx_buf[idx][5] << 8)  |
                        (uint32_t)can1.rx_buf[idx][6];

        float Motor_Angle = (can1.rx_buf[idx][2] == 0x01) ?
                            -((float)Pos * 360.0f / 65535.0f) : ((float)Pos * 360.0f / 65535.0f);

        static const float ratios[] = {50.0f, 51.0f, 51.0f, 51.0f, 27.0f, 51.0f};
        float Joint_Angle = Motor_Angle / ratios[idx];

        if (motor_id==3||motor_id==4)
        {
          can1.current_angle[idx] = -(Joint_Angle * M_PI / 180.0f) + dh_theta[idx];
        }
        else
        {
          can1.current_angle[idx] = (Joint_Angle * M_PI / 180.0f) + dh_theta[idx];
        }

      }
    }
  }
  else if (pack_num == 1) {
    // 拼接第二包到对应电机的抽屉
    if (can1.rx_buf[idx][0] == motor_id) {
      for (uint8_t i = 0; i < pHeader->DLC; i++) {
        can1.rx_buf[idx][i + 9] = pData[i];
      }
    }
  }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
      CAN_Process_Motor_Data(&header, data, 0);
    }
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) == HAL_OK) {
      CAN_Process_Motor_Data(&header, data, 1);
    }
  }
}



// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   if(hcan->Instance == CAN1)
//   {
//     CAN_RxHeaderTypeDef CAN_RxMsg;
//     uint8_t rx_data[8]={0};
//     if (HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxMsg,rx_data)==HAL_OK)
//     {
//       if ((CAN_RxMsg.ExtId&0xFF)==0)
//       {
//         can1.rx_buf[0]=CAN_RxMsg.ExtId>>8;
//         for (uint8_t i=0;i<CAN_RxMsg.DLC;i++)
//         {
//           can1.rx_buf[i+1]=rx_data[i];
//         }
//         if (CAN_RxMsg.DLC<8)
//         {
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+1,1000);
//           if (can1.rx_buf[1] == 0x36) // 功能码
//           {
//             can1.rx_flag[can1.rx_buf[0]-1]=1;
//             uint32_t Pos=can1.rx_buf[3]<<24|can1.rx_buf[4]<<16|can1.rx_buf[5]<<8|can1.rx_buf[6];
//
//             float Motor_Angle;
//             if (can1.rx_buf[2] == 0x01) // 负数
//             {
//               Motor_Angle = -((float)Pos * 360.0f / 65535.0f);
//             }
//             else // 正数
//             {
//               Motor_Angle = ((float)Pos * 360.0f / 65535.0f);
//             }
//
//             float Joint_Angle = 0;
//
//             switch (can1.rx_buf[0])
//             {
//             case 0x01: Joint_Angle = Motor_Angle / 50; break;
//             case 0x02: Joint_Angle = Motor_Angle / 51; break;
//             case 0x03: Joint_Angle = Motor_Angle / 51; break;
//             case 0x04: Joint_Angle = Motor_Angle / 51; break;
//             case 0x05: Joint_Angle = Motor_Angle / 27; break;
//             case 0x06: Joint_Angle = Motor_Angle / 51; break;
//             default: break;
//             }
//
//             can1.current_angle[can1.rx_buf[0]-1]=(Joint_Angle * M_PI / 180.0f) + dh_theta[can1.rx_buf[0]-1];
//
//           }
//         }
//         else if (CAN_RxMsg.DLC==8&&can1.rx_buf[7]==0x6B)
//         {
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+1,1000);
//         }
//
//       }
//       else if ((CAN_RxMsg.ExtId&0xFF)==1)
//       {
//         if (can1.rx_buf[0]==CAN_RxMsg.ExtId>>8)
//         {
//           for (uint8_t i=0;i<CAN_RxMsg.DLC;i++)
//           {
//             can1.rx_buf[i+9]=rx_data[i+1];
//           }
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+8,1000);
//         }
//       }
//     }
//   }
// }
//
// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   if(hcan->Instance == CAN1)
//   {
//     CAN_RxHeaderTypeDef CAN_RxMsg;
//     uint8_t rx_data[8]={0};
//     if (HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO1,&CAN_RxMsg,rx_data)==HAL_OK)
//     {
//       uint8_t rx_buf[32];
//       if ((CAN_RxMsg.ExtId&0xFF)==0)
//       {
//         rx_buf[0]=CAN_RxMsg.ExtId>>8;
//         for (uint8_t i=0;i<CAN_RxMsg.DLC;i++)
//         {
//           rx_buf[i+1]=rx_data[i];
//         }
//         if (CAN_RxMsg.DLC<8)
//         {
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+1,1000);
//           if (rx_buf[1] == 0x36) // 功能码
//           {
//             can1.rx_flag[rx_buf[0]-1]=1;
//             uint32_t Pos=rx_buf[3]<<24|rx_buf[4]<<16|rx_buf[5]<<8|rx_buf[6];
//
//             float Motor_Angle;
//             if (rx_buf[2] == 0x01) // 负数
//             {
//               Motor_Angle = -((float)Pos * 360.0f / 65535.0f);
//             }
//             else // 正数
//             {
//               Motor_Angle = ((float)Pos * 360.0f / 65535.0f);
//             }
//
//             float Joint_Angle = 0;
//
//             switch (rx_buf[0])
//             {
//             case 0x01: Joint_Angle = Motor_Angle / 50; break;
//             case 0x02: Joint_Angle = Motor_Angle / 51; break;
//             case 0x03: Joint_Angle = Motor_Angle / 51; break;
//             case 0x04: Joint_Angle = Motor_Angle / 51; break;
//             case 0x05: Joint_Angle = Motor_Angle / 27; break;
//             case 0x06: Joint_Angle = Motor_Angle / 51; break;
//             default: break;
//             }
//
//             can1.current_angle[rx_buf[0]-1]=(Joint_Angle * M_PI / 180.0f) + dh_theta[rx_buf[0]-1];
//
//           }
//         }
//         else if (CAN_RxMsg.DLC==8&&rx_buf[7]==0x6B)
//         {
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+1,1000);
//         }
//
//       }
//       else if ((CAN_RxMsg.ExtId&0xFF)==1)
//       {
//         if (rx_buf[0]==CAN_RxMsg.ExtId>>8)
//         {
//           for (uint8_t i=0;i<CAN_RxMsg.DLC;i++)
//           {
//             rx_buf[i+9]=rx_data[i+1];
//           }
//           // HAL_UART_Transmit(&huart1,can1.rx_buf,CAN_RxMsg.DLC+8,1000);
//         }
//       }
//     }
//   }
// }
/* USER CODE END 1 */

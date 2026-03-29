//
// Created by 57500 on 2026/3/10.
//

#ifndef MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H
#define MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H

// c_hal_bridge.h
// 所有 C++ 文件统一通过这个文件引入 HAL 和外设头文件
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//顺序
#include "stm32f4xx_hal.h"

#include "main.h"
#include "can.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#ifdef __cplusplus
}
#endif


#endif //MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H
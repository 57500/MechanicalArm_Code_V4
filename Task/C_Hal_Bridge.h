//
// Created by 57500 on 2026/3/10.
//

#ifndef MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H
#define MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H

// c_hal_bridge.h
// 所有 C++ 文件统一通过这个文件引入 HAL 和外设头
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    // ── HAL 核心（顺序很重要，不能乱）──
#include "stm32f4xx_hal.h"   // 一次性引入全部 HAL，不要单独引子模块

    // ── CubeMX 生成的外设初始化头 ──
#include "main.h"
#include "can.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

    // ── FreeRTOS ──
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#ifdef __cplusplus
}
#endif


#endif //MECHANICALARM_CODE_V3_C_HAL_BRIDGE_H
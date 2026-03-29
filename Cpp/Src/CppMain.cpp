// //
// // Created by 57500 on 2026/2/27.
// //
#include "CppMain.h"

#include <cstdio>
#include <cstring>

#include "can.h"
#include "MotorControl.h"
#include "Forward_Kinematics.h"
#include "Inverse_Kinematics.h"
#include "RobotState.h"
#include "Trajectory_Planning.h"
#include "usart.h"
#include "main.h"
#include "NewBee.h"
#include "Robot_Task.h"

// 启用DWT
void Init_DWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// //测时专用
// DWT->CYCCNT = 0;  // 清零计数器
// uint32_t start_cycles = DWT->CYCCNT;
//
// //->测时内容
//
// uint32_t end_cycles = DWT->CYCCNT;
// uint32_t total_cycles = end_cycles - start_cycles;
//
// // 转换为时间（单位：微秒）
// float time_us = (float)total_cycles * 1000000.0f / SystemCoreClock;
// // 或者毫秒
// float time_ms = (float)total_cycles * 1000.0f / SystemCoreClock;
//
// // 输出结果
// char buffer[64];
// sprintf(buffer, "执行时间: %.3f ms (周期数: %lu)\r\n", time_ms, total_cycles);
// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);



//握手函数
void CppMain(void)
{
    Init_DWT();
    RobotTask_Init();
}

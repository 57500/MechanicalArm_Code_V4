//
// Created by 57500 on 2026/3/10.
//

#ifndef MECHANICALARM_CODE_V3_ROBOT_TASK_H
#define MECHANICALARM_CODE_V3_ROBOT_TASK_H
#include "FreeRTOS.h"
#include "NewBee.h"
#include "semphr.h"
#include "task.h"

extern NewBee nb;

[[noreturn]] void PTP_Task(void *pvParameters);
[[noreturn]] void PAD_Task(void *pvParameters);
void Uart_Task(void *pvParameters);
void RobotTask_Init(void);

extern TaskHandle_t PTP_Handle;
extern TaskHandle_t PAD_Handle;
extern TaskHandle_t Uart_Handle;
extern TaskHandle_t Current_Task;

extern QueueHandle_t Uart_Queue_H;
extern QueueHandle_t Tim1_Queue_H;
extern QueueHandle_t Pad_Queue_H;

extern SemaphoreHandle_t Stop_Semaphore_H;

extern volatile bool Stop_Flag;


#endif //MECHANICALARM_CODE_V3_ROBOT_TASK_H
//
// Created by 57500 on 2026/3/20.
//

#include <cstdio>
#include <cstring>

#include "Robot_Task.h"
#include "tim.h"
#include "usart.h"

static Pad_Params_t pd;

static uint8_t TimFlag=0;

/**
 * @brief PAD任务100hz
 * @param NULL
 * @return NULL
 */
[[noreturn]] void PAD_Task(void *pvParameters)
{
    //当切换到别的任务时在此处等待
    Wait:
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (1)
    {
        //等待定时器一队列
        BaseType_t Receive=xQueueReceive(Tim1_Queue_H,&TimFlag,pdMS_TO_TICKS(20));

        //判断是否需要切换任务
        if (Stop_Flag==true)
        {
            Stop_Flag = false;
            nb.Clear();
            nb.Emergency_Stop();
            HAL_TIM_Base_Stop_IT(&htim1);
            xSemaphoreGive(Stop_Semaphore_H);
            xQueueReset(Tim1_Queue_H);

            goto Wait;
        }

        //等待串口手柄数据
        BaseType_t Date_Receive=xQueueReceive(Pad_Queue_H,&pd,pdMS_TO_TICKS(20));

        //执行一次手柄控制
        if (Receive==pdTRUE)
        {
            if (Stop_Flag==false)
            {
                if (Date_Receive==pdTRUE)
                {
                    nb.UpDate_Current_Pad(pd);
                    nb.UpDate_Pad_Control();
                }
            }
        }
    }
}


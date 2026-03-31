//
// Created by 57500 on 2026/3/10.
//

#include <cstdio>
#include <cstring>

#include "Robot_Task.h"
#include "C_Hal_Bridge.h"

static uint8_t TimFlag=0;
/**
 * @brief PAD任务100hz
 * @param NULL
 * @return NULL
 */
[[noreturn]] void PTP_Task(void *pvParameters)
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

        //执行一次PTP控制
        if (Receive==pdTRUE)
        {
            if (Stop_Flag==false)
            {
                if (nb.Get_Current_Step()>0)
                {
                    nb.UpDate_PTP_Control();
                }
                else if (nb.Get_Current_Step()==0)
                {
                    nb.Emergency_Stop();
                }
            }
        }
    }
}

/**
 * @brief 定时器一回调函数100hz
 * @param 定时器句柄
 * @return NULL
 */
extern "C" void CPP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        BaseType_t xHigherPriorityTaskWoken=pdFALSE;
        uint8_t TimFlag=1;
        xQueueSendFromISR(Tim1_Queue_H,&TimFlag,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

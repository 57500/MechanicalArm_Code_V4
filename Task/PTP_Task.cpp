//
// Created by 57500 on 2026/3/10.
//

#include <cstdio>
#include <cstring>

#include "Robot_Task.h"
#include "C_Hal_Bridge.h"



[[noreturn]] void PTP_Task(void *pvParameters)
{

    Wait:
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


    while (1)
    {
        uint8_t TimFlag=0;

        BaseType_t Receive=xQueueReceive(Tim1_Queue_H,&TimFlag,pdMS_TO_TICKS(20));

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

        if (Receive==pdTRUE)
        {
            if (Stop_Flag==false)
            {
                if (nb.Get_Current_Step()>0)
                {
                    nb.UpDate();
                    uint8_t a=1;
                }
            }
        }
    }
}

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

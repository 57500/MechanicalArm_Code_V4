//
// Created by 57500 on 2026/3/20.
//

#include "Robot_Task.h"
#include "tim.h"

static Pad_Params_t pd;

[[noreturn]] void PAD_Task(void *pvParameters)
{
    Wait:
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


    while (1)
    {
        uint8_t TimFlag=0;

        BaseType_t Receive=xQueueReceive(Tim1_Queue_H,&TimFlag,pdMS_TO_TICKS(20));

        BaseType_t Date_Receive=xQueueReceive(Pad_Queue_H,&pd,pdMS_TO_TICKS(20));

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
                if (Date_Receive==pdTRUE)
                {
                    nb.UpDate_Current_Pad(pd);
                    nb.UpDate_Pad_Control();
                }

            }
        }
    }
}

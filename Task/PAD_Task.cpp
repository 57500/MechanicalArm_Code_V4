//
// Created by 57500 on 2026/3/20.
//

#include "Robot_Task.h"

[[noreturn]] void PAD_Task(void *pvParameters)
{
    Wait:
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


    while (1)
    {
        uint8_t TimFlag=0;

        BaseType_t Receive=xQueueReceive(Tim1_Queue_H,&TimFlag,pdMS_TO_TICKS(20));

        if (Stop_Flag==true)
        {
            nb.Clear();
            nb.Emergency_Stop();
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
                }
            }
        }
    }
}
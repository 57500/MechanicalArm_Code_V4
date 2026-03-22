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

    }
}
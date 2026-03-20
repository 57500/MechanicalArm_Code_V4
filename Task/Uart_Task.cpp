//
// Created by 57500 on 2026/3/10.
//
#include <cstring>

#include "C_Hal_Bridge.h"
#include "Robot_Task.h"


void Uart_Task(void *pvParameters)
{
    uint8_t Date[RX_BUFFER_SIZE1];

    while(1)
    {
        if (xQueueReceive(Uart_Queue_H, Date, portMAX_DELAY) != pdTRUE) continue;

        if (Current_Task != NULL)
        {
            Stop_Flag = true;

            if (xSemaphoreTake(Stop_Semaphore_H, pdMS_TO_TICKS(200)) != pdTRUE)
            {
                nb.Clear();
                nb.Emergency_Stop();
                Current_Task = NULL;
                HAL_TIM_Base_Start_IT(&htim1);
                HAL_UART_Transmit(&huart1,(uint8_t*)"StopSemaphore Miss",sizeof("StopSemaphore Miss"),100);
            }
        }
        Stop_Flag = false;

        if (strncmp((char*)Date, "PTP:", 4) == 0)
        {
            char *token;
            int count = 0;
            float temp[6];

            // 以此逗号 "," 为分隔符获取第一个数据
            token = strtok((char*)Date+4, ",");

            while(token != NULL && count < 6)
            {
                //将字符串转为浮点数
                temp[count] = atof(token);
                count++;

                //以此逗号 "," 为分隔符获取下一个数据
                token = strtok(NULL, ",");
            }

            if (count == 6)
            {
                Coordinates_Pose target;
                target.x=temp[0];
                target.y=temp[1];
                target.z=temp[2];
                target.alpha=temp[3];
                target.beta=temp[4];
                target.gamma=temp[5];

                nb.UpDate_Current_Angle_Rad();
                nb.UpDate_Current_CP();
                nb.UpDate_Target_CP(target);
                nb.UpDate_S_Curve_Profile();

                Current_Task=PTP_Handle;
                HAL_TIM_Base_Start_IT(&htim1);
                xTaskNotifyGive(Current_Task);
            }
        }

    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (uart1.rx_byte == '\n')
        {
            uart1.rx_buffer[uart1.rx_index] = '\0'; // 字符串结束符
            uart1.rx_index = 0;               // 索引归零

            xQueueSendFromISR(Uart_Queue_H,uart1.rx_buffer,&xHigherPriorityTaskWoken);

        }
        else
        {
            // 存放
            if (uart1.rx_index < RX_BUFFER_SIZE1 - 1) // 防止数组越界
            {
                uart1.rx_buffer[uart1.rx_index++] = uart1.rx_byte;
            }
            else
            {
                uart1.rx_index = 0;
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &uart1.rx_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

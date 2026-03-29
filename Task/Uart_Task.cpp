//
// Created by 57500 on 2026/3/10.
//
#include <cstring>


#include "cmsis_os2.h"
#include "C_Hal_Bridge.h"
#include "Robot_Task.h"

//记录上一次任务
static TaskName LastTask=PTP;

//串口数据缓存
static uint8_t Date[RX_BUFFER_SIZE1];

void Uart_Task(void *pvParameters)
{
    while(1)
    {
        //等待串口数据
        if (xQueueReceive(Uart_Queue_H, Date, portMAX_DELAY) != pdTRUE) continue;

        TaskName CurrentTask = NONE;

        //解析当前任务模式名称
        if (strncmp((char*)Date, "PTP:", 4) == 0) CurrentTask = PTP;
        else if (strncmp((char*)Date, "PAD:", 4) == 0) CurrentTask = PAD;
        else continue;

        //判断是否需要切换任务前进行急停，如果是PTP任务则一定急停
        if ((CurrentTask != LastTask && LastTask != NONE)||(CurrentTask==PTP&&LastTask!=NONE))
        {
            Stop_Flag = true;

            if (xSemaphoreTake(Stop_Semaphore_H, pdMS_TO_TICKS(200)) != pdTRUE)
            {
                nb.Clear();
                nb.Emergency_Stop();
                Current_Task = NULL;
                HAL_TIM_Base_Stop_IT(&htim1);
                HAL_UART_Transmit(&huart1, (uint8_t*)"StopSemaphore Miss", sizeof("StopSemaphore Miss"), 100);
            }
        }

        //解析PTP任务数据
        if (CurrentTask==PTP)
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

                //防止总线忙碌！！！！！！
                osDelay(10);

                nb.UpDate_Current_Angle_Rad();
                nb.UpDate_Current_CP();
                nb.UpDate_Target_CP(target);
                nb.UpDate_S_Curve_Profile();
            }
        }
        else if (CurrentTask==PAD)
        {
            char *token;
            int count = 0;
            int16_t pad_data[16] = {0}; // 用于存放 16 个手柄整型数据

            token = strtok((char*)Date + 4, ",");

            // 循环分割字符串，最多提取 16 个参数
            while(token != NULL && count < 16)
            {
                // 使用 atoi 将字符串转为整数
                pad_data[count] = atoi(token);
                count++;

                // 获取下一个数据
                token = strtok(NULL, ",");
            }

            if (count==16)
            {
                Pad_Params_t Pad_Params;
                Pad_Params.lx=pad_data[0];
                Pad_Params.ly=pad_data[1];
                Pad_Params.rx=pad_data[2];
                Pad_Params.ry=pad_data[3];
                Pad_Params.lt=pad_data[4];
                Pad_Params.rt=pad_data[5];
                Pad_Params.btn_a=pad_data[6];
                Pad_Params.btn_b=pad_data[7];
                Pad_Params.btn_x=pad_data[8];
                Pad_Params.btn_y=pad_data[9];
                Pad_Params.btn_lb=pad_data[10];
                Pad_Params.btn_rb=pad_data[11];
                Pad_Params.dpad_x=pad_data[12];
                Pad_Params.dpad_y=pad_data[13];
                Pad_Params.btn_m1=pad_data[14];
                Pad_Params.btn_m2=pad_data[15];

                //发送数据
                xQueueSend(Pad_Queue_H,&Pad_Params,pdMS_TO_TICKS(20));

            }
        }

        //判断是否需要切换任务，如果是PTP任务则再次进入PTP
        if ((CurrentTask != LastTask && LastTask != NONE)||(CurrentTask==PTP&&LastTask!=NONE))
        {
            if (CurrentTask == PTP)
            {
                Current_Task=PTP_Handle;
                HAL_TIM_Base_Start_IT(&htim1);
            }
            else if (CurrentTask == PAD)
            {
                Current_Task=PAD_Handle;
                HAL_TIM_Base_Start_IT(&htim1);
            }

            xTaskNotifyGive(Current_Task);
        }

        //记录上一次任务名称
        LastTask=CurrentTask;

    }
}

/**
 * @brief 串口回调哈桑农户
 * @param 串口句柄
 * @return NULL
 */
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
        __HAL_UART_CLEAR_OREFLAG(huart);

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &uart1.rx_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


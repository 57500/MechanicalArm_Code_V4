//
// Created by 57500 on 2026/3/10.
//
#include "Robot_Task.h"
#include "usart.h"

NewBee nb;

TaskHandle_t PTP_Handle;
static StaticTask_t PTP_Tcb;
static StackType_t PTP_Stack[1024];

TaskHandle_t Uart_Handle;
static StaticTask_t Uart_Tcb;
static StackType_t Uart_Stack[512];

QueueHandle_t Uart_Queue_H;
static StaticQueue_t Uart_Queue_Tcb;
static uint8_t Uart_Queue_Stack[5*RX_BUFFER_SIZE1];

QueueHandle_t Tim1_Queue_H;
static StaticQueue_t Tim1_Queue_Tcb;
static uint8_t Tim1_Queue_Stack[5*sizeof(uint8_t)];

static StaticSemaphore_t Stop_Semaphore_Stack;
SemaphoreHandle_t Stop_Semaphore_H;

TaskHandle_t Current_Task=NULL;
volatile bool Stop_Flag=false;

void RobotTask_Init(void)
{
    Stop_Semaphore_H=xSemaphoreCreateBinaryStatic(&Stop_Semaphore_Stack);

    Uart_Queue_H=xQueueCreateStatic(5,RX_BUFFER_SIZE1,Uart_Queue_Stack,&Uart_Queue_Tcb);

    Tim1_Queue_H = xQueueCreateStatic(5, sizeof(uint8_t), Tim1_Queue_Stack, &Tim1_Queue_Tcb);

    Uart_Handle=xTaskCreateStatic(Uart_Task,"Uart",512,
        NULL,1,Uart_Stack,&Uart_Tcb);

    PTP_Handle=xTaskCreateStatic(PTP_Task,"PTP",1024,
        NULL,3,PTP_Stack,&PTP_Tcb);


}



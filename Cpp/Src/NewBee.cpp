//
// Created by 57500 on 2026/3/2.
//

#include "NewBee.h"

#include <cstdio>
#include <cstring>

#include "can.h"
#include "usart.h"

NewBee::NewBee()
    : motor1(1), motor2(2), motor3(3), motor4(4), motor5(5), motor6(6)
{

}

void NewBee::Clear()
{
    ik.Clear();
    state.Clear();
    tp.Clear();
    pd.Clear();
}

void NewBee::UpDate()
{
    UpDate_Current_Angle_Rad();
    ik.Solve_FinalTheta(tp.CP_Ref[tp.Count],state.Current_Angle_Rad,state.Next_Best_Angle_Rad);
    tp.Control_Once(state.Current_Angle_Rad,state.Next_Best_Angle_Rad);

    Control_All_Motor(tp.JointRPM);

    // UpDate_Current_CP();
    // float x[2]={tp.CP_Ref[tp.Count].x,state.Current_CP.x};
    // Send_FireWater_Text(x,2);
}

void NewBee::UpDate_Pad_Control()
{
    Check_Pad_Mode();

    Check_Pad_Sensitivity();

    Check_Pad_Lock();

    Check_Pad_Home();

    if (state.Pad_Home==false)
    {
        if (state.Current_Pad_Mode==Cartesian)
        {
            UpDate_Current_Angle_Rad();
            UpDate_Current_CP();

            pd.Calculate_Target_CP(state.Current_Pad,state.Current_CP,state.Current_Pad_Lock,state.Pad_Cartesian_Sensitivity);
            ik.Solve_FinalTheta(pd.Target_CP,state.Current_Angle_Rad,state.Next_Best_Angle_Rad);
            pd.Control_Once(state.Current_Angle_Rad,state.Next_Best_Angle_Rad);

            Control_All_Motor(pd.JointRPM);
        }
        else if (state.Current_Pad_Mode==Joint)
        {
            pd.Calculate_Target_Joint(state.Current_Pad,state.Current_Pad_Lock,state.Pad_Cartesian_Sensitivity);

            Control_All_Motor(pd.JointRPM);
        }
        else if (state.Current_Pad_Mode==Tool)
        {
            UpDate_Current_Angle_Rad();
            UpDate_Current_CP();

            pd.Calculate_Target_ToolCP(state.Current_Pad,state.Current_CP,state.Pad_Cartesian_Sensitivity,ik.ZYZ_to_RotationMatrix(state.Current_CP));
            ik.Solve_FinalTheta(pd.Target_CP,state.Current_Angle_Rad,state.Next_Best_Angle_Rad);
            pd.Control_Once(state.Current_Angle_Rad,state.Next_Best_Angle_Rad);

            Control_All_Motor(pd.JointRPM);
        }
    }
    else
    {
        if (state.is_first_home)
        {
            UpDate_Current_Angle_Rad();
            UpDate_Current_CP();
            UpDate_Target_CP(Home);
            UpDate_S_Curve_Profile();
            state.is_first_home=false;
        }
        else
        {
            if (Get_Current_Step()>0)
            {
                UpDate();
            }
            else
            {
                float JointRPM[6] = {0};
                Control_All_Motor(JointRPM);

                tp.Clear();
                state.is_first_home=true;
                state.Pad_Home = false;
            }
        }
    }


    state.Last_Pad=state.Current_Pad;
}

void NewBee::Control_All_Motor(const float* rpm)
{
    motor1.Motor_VelControl(rpm[0],0,0);
    motor2.Motor_VelControl(rpm[1],0,0);
    motor3.Motor_VelControl(rpm[2],0,0);
    motor4.Motor_VelControl(rpm[3],0,0);
    motor5.Motor_VelControl(rpm[4],0,0);
    motor6.Motor_VelControl(rpm[5],0,0);
}

void NewBee::Emergency_Stop(void)
{
    Clear();

    motor1.Motor_VelControl(0,0,0);
    motor2.Motor_VelControl(0,0,0);
    motor3.Motor_VelControl(0,0,0);
    motor4.Motor_VelControl(0,0,0);
    motor5.Motor_VelControl(0,0,0);
    motor6.Motor_VelControl(0,0,0);
}

void NewBee::UpDate_Current_Angle_Rad(void)
{
    //并行通讯
    for (int i=0; i<6; i++)
    {
        uint32_t wait_start = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        {
            if (HAL_GetTick() - wait_start > 2)
            {
                break;
            }
        }
        MotorControl::Read_Motor_Pos(i+1);
    }

    uint32_t start_tick = HAL_GetTick();
    while (!(can1.rx_flag[0] && can1.rx_flag[1] && can1.rx_flag[2] &&
         can1.rx_flag[3] && can1.rx_flag[4]&&can1.rx_flag[5]))
    {
        if (HAL_GetTick() - start_tick > 6)
        {
            break;
        }
    }

    for (int i=0; i<6; i++)
    {
        state.Current_Angle_Rad[i]=can1.current_angle[i];
        can1.rx_flag[i] = 0;
    }


    ////串行通讯
    // for (int i = 0; i < 6; i++)
    // {
    //     DWT->CYCCNT = 0;  // 清零计数器
    //     uint32_t start_cycles = DWT->CYCCNT;
    //
    //
    //     MotorControl::Read_Motor_Pos(i);
    //     uint32_t start_tick = HAL_GetTick();
    //     while (can1.rx_flag!=1)
    //     {
    //         if (HAL_GetTick()-start_tick > 5)break;
    //     }
    //
    //     uint16_t Pos=can1.rx_buf[3]<<24|can1.rx_buf[4]<<16|can1.rx_buf[5]<<8|can1.rx_buf[6];
    //
    //     uint32_t end_cycles = DWT->CYCCNT;
    //     uint32_t total_cycles = end_cycles - start_cycles;
    //
    //     // 转换为时间（单位：微秒）
    //     float time_us = (float)total_cycles * 1000000.0f / SystemCoreClock;
    //     // 或者毫秒
    //     float time_ms = (float)total_cycles * 1000.0f / SystemCoreClock;
    //
    //     // 输出结果
    //     char buffer[64];
    //     sprintf(buffer, "执行时间: %.3f ms (周期数: %lu)\r\n", time_ms, total_cycles);
    //     HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    //
    //     float Motor_Angle;
    //     if (can1.rx_buf[2] == 0x01) // 负数
    //     {
    //         Motor_Angle = -((float)Pos * 360.0f / 65535.0f);
    //     }
    //     else // 正数
    //     {
    //         Motor_Angle = ((float)Pos * 360.0f / 65535.0f);
    //     }
    //
    //     float Joint_Angle = 0;
    //
    //     switch (can1.rx_buf[0])
    //     {
    //     case 0x01: Joint_Angle = Motor_Angle / 50; break;
    //     case 0x02: Joint_Angle = Motor_Angle / 51; break;
    //     case 0x03: Joint_Angle = Motor_Angle / 51; break;
    //     case 0x04: Joint_Angle = Motor_Angle / 51; break;
    //     case 0x05: Joint_Angle = Motor_Angle / 27; break;
    //     case 0x06: Joint_Angle = Motor_Angle / 51; break;
    //     default: break;
    //     }
    //
    //     Current_Angle_Rad[i]=(Joint_Angle * PI / 180.0f) + DH_Params[can1.rx_buf[0]-1].theta;
    //
    //     can1.rx_flag=0;
    // }
}

float* NewBee::Get_Current_Angle_Rad(void)
{
    return state.Current_Angle_Rad;
}

void NewBee::UpDate_S_Curve_Profile(void)
{
    tp.S_Curve_Profile(state.Target_CP, state.Current_CP);
}

uint32_t NewBee::Get_Current_Step(void)
{
    return tp.Step;
}

void NewBee::UpDate_Target_CP(const Coordinates_Pose& target_cp)
{
    state.Target_CP=target_cp;
}

void NewBee::UpDate_Current_CP()
{
    state.Current_CP=fk.Forward_Kinematics(state.Current_Angle_Rad);
}

void NewBee::UpDate_Current_Pad(const Pad_Params_t pd)
{
    state.Current_Pad=pd;
}

void NewBee::Check_Pad_Mode(void)
{
    if (state.Current_Pad.btn_y==1&&state.Last_Pad.btn_y==0)
    {
        state.Current_Pad_Mode = static_cast<Pad_Mode>((state.Current_Pad_Mode + 1) % MODE_COUNT);
        pd.Clear();
        Control_All_Motor(pd.JointRPM);

        switch (state.Current_Pad_Mode)
        {
        case Standby:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Standby\n",strlen("Mode: Standby\n"),100);
            break;

        case Cartesian:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Cartesian\n",strlen("Mode: Cartesian\n"),100);
            break;

        case Joint:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Joint\n",strlen("Mode: Joint\n"),100);
            break;

        case Tool:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Tool\n",strlen("Mode: Tool\n"),100);
            break;

        default:
            break;
        }
    }
}

void NewBee::Check_Pad_Sensitivity(void)
{
    if (state.Current_Pad.btn_x==1&&state.Last_Pad.btn_x==0)
    {
        if (state.Pad_Cartesian_Sensitivity<=2)
        {
            state.Pad_Cartesian_Sensitivity+=0.1f;
        }
        else
        {
            state.Pad_Cartesian_Sensitivity=0.1f;
        }

        char buffer[64];
        sprintf(buffer, "Sensitivity: %.1f \r\n", state.Pad_Cartesian_Sensitivity);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

void NewBee::Check_Pad_Lock()
{
    if (state.Current_Pad.btn_b==1&&state.Last_Pad.btn_b==0)
    {
        state.Current_Pad_Lock = static_cast<Pad_Lock>((state.Current_Pad_Lock + 1) % LOCK_COUNT);
        pd.Clear();
        Control_All_Motor(pd.JointRPM);

        switch (state.Current_Pad_Lock)
        {
        case None:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: None\n",strlen("Mode: None\n"),100);
            break;

        case Lock1:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock1\n",strlen("Mode: Lock1\n"),100);
            break;

        case Lock2:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock2\n",strlen("Mode: Lock2\n"),100);
            break;

        case Lock3:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock3\n",strlen("Mode: Lock3\n"),100);
            break;

        case Lock4:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock4\n",strlen("Mode: Lock4\n"),100);
            break;

        case Lock5:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock5\n",strlen("Mode: Lock5\n"),100);
            break;

        case Lock6:
            HAL_UART_Transmit(&huart1,(uint8_t*)"Mode: Lock6\n",strlen("Mode: Lock6\n"),100);
            break;

        default:
            break;
        }
    }
}

void NewBee::Check_Pad_Home(void)
{
    if (state.Current_Pad.btn_a==1&&state.Last_Pad.btn_a==0)
    {
        pd.Clear();
        Control_All_Motor(pd.JointRPM);
        state.is_first_home = true;
        tp.Clear();
        state.Pad_Home=!state.Pad_Home;

        HAL_UART_Transmit(&huart1,(uint8_t*)"Home\n",strlen("Home\n"),100);
    }
}
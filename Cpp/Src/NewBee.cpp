//
// Created by 57500 on 2026/3/2.
//

#include "NewBee.h"

#include <cstdio>
#include <cstring>

#include "can.h"
#include "usart.h"

/**
 * @brief 初始化
 * @param NULL
 * @return NULL
 */
NewBee::NewBee()
    : motor1(1), motor2(2), motor3(3), motor4(4), motor5(5), motor6(6)
{
    Clear();
}

/**
 * @brief 重置整个NewBee
 * @param NULL
 * @return NULL
 */
void NewBee::Clear()
{
    ik.Clear();
    state.Clear();
    tp.Clear();
    pd.Clear();
}

/**
 * @brief 执行一次PTP任务
 * @param NULL
 * @return NULL
 */
void NewBee::UpDate_PTP_Control()
{
    UpDate_Current_Angle_Rad();
    ik.Solve_FinalTheta(tp.CP_Ref[tp.Count],state.Current_Angle_Rad,state.Next_Best_Angle_Rad);
    tp.Control_Once(state.Current_Angle_Rad,state.Next_Best_Angle_Rad);

    Control_All_Motor(tp.JointRPM);

    float x[5]={state.Next_Best_Angle_Rad[4],state.Current_Angle_Rad[4],tp.JointRPM[4],(float)tp.Step,tp.CP_Ref[tp.Count].beta};
    Send_FireWater_Text(x,5);
}

/**
 * @brief 执行一次PAD任务
 * @param NULL
 * @return NULL
 */
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



            // float x[4]={state.Next_Best_Angle_Rad[5],state.Current_Angle_Rad[5],pd.JointRPM[5]};
            // Send_FireWater_Text(x,4);
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
                UpDate_PTP_Control();
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

/**
 * @brief 控制所有电机
 * @param rpm：关节转速
 * @return NULL
 */
void NewBee::Control_All_Motor(const float* rpm)
{
    motor1.Motor_VelControl(rpm[0],0,0);
    motor2.Motor_VelControl(rpm[1],0,0);
    motor3.Motor_VelControl(rpm[2],0,0);
    motor4.Motor_VelControl(rpm[3],0,0);
    motor5.Motor_VelControl(rpm[4],0,0);
    motor6.Motor_VelControl(rpm[5],0,0);
}

/**
 * @brief 急停
 * @param NULL
 * @return NULL
 */
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

/**
 * @brief 更新当前关节角度
 * @param NULL
 * @return NULL
 */
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
}

/**
 * @brief 获取内部关节角度
 * @param NULL
 * @return NULL
 */
float* NewBee::Get_Current_Angle_Rad(void)
{
    return state.Current_Angle_Rad;
}

/**
 * @brief 更新S型曲线
 * @param NULL
 * @return NULL
 * @note 要确保调用前已经更新了目标位姿和当前位姿
 */
void NewBee::UpDate_S_Curve_Profile(void)
{
    tp.S_Curve_Profile(state.Target_CP, state.Current_CP);
}

/**
 * @brief 获取内部轨迹规划步数
 * @param NULL
 * @return 步数
 */
uint32_t NewBee::Get_Current_Step(void)
{
    return tp.Step;
}

/**
 * @brief 更新目标位姿到内部
 * @param 目标位姿
 * @return NULL
 */
void NewBee::UpDate_Target_CP(const Coordinates_Pose& target_cp)
{
    state.Target_CP=target_cp;
}

/**
 * @brief 根据内部当前角度计算当前位姿
 * @param NULL
 * @return NULL
 */
void NewBee::UpDate_Current_CP()
{
    state.Current_CP=fk.Forward_Kinematics(state.Current_Angle_Rad);
}

/**
 * @brief 更新当前手柄参数到内部
 * @param pd：当前手柄参数
 * @return NULL
 */
void NewBee::UpDate_Current_Pad(const Pad_Params_t pd)
{
    state.Current_Pad=pd;
}

/**
 * @brief 检查是否需要更换手柄模式
 * @param NULL
 * @return NULL
 */
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

/**
 * @brief 检查是否需要修改手柄灵敏度
 * @param NULL
 * @return NULL
 */
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

/**
 * @brief 检查是否需要修改手柄锁定信息
 * @param NULL
 * @return NULL
 */
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

/**
 * @brief 检查是否回家
 * @param NULL
 * @return NULL
 */
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
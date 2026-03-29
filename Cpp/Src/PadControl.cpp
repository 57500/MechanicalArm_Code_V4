//
// Created by 57500 on 2026/3/22.
//

#include "PadControl.h"

#include <cmath>
#include <cstring>

#include "usart.h"

PadControl::PadControl()
{
    Clear();
}

void PadControl::Clear()
{
    memset(JointRPM,0,sizeof(float)*6);
    Target_CP={};
    Target_PD = {};
    Last_PD={};

    smooth_vx = 0.0f;
    smooth_vy = 0.0f;
    smooth_vz = 0.0f;
    smooth_w_alpha = 0.0f;
    smooth_w_beta = 0.0f;
    smooth_w_gamma = 0.0f;

    smooth_joint1 = 0.0f;
    smooth_joint2 = 0.0f;
    smooth_joint3 = 0.0f;
    smooth_joint4 = 0.0f;
    smooth_joint5 = 0.0f;
    smooth_joint6 = 0.0f;

    smooth_tool_z = 0.0f;
}

void PadControl::Calculate_Target_CP(const Pad_Params_t Current_PD, const Coordinates_Pose Current_CP, const Pad_Lock PL, const float Sensitivity)
{
    // 1. 摇杆死区消除 (Deadzone) - 极其重要！
    // 手柄回弹很难绝对回0，一般在 -5 到 5 之间波动，不加死区机器人会一直漂移
    const float DEADZONE = 10.0f;

    float raw_lx = (fabs(Current_PD.lx) > DEADZONE) ? Current_PD.lx : 0.0f;
    float raw_ly = (fabs(Current_PD.ly) > DEADZONE) ? Current_PD.ly : 0.0f;
    float raw_rx = (fabs(Current_PD.rx) > DEADZONE) ? Current_PD.rx : 0.0f;
    float raw_ry = (fabs(Current_PD.ry) > DEADZONE) ? Current_PD.ry : 0.0f;

    // 2. 计算当前的“期望速度” (Target Velocity)
    // 注意：这里只是算出了目标值，还没有加到坐标上
    float target_vx = -raw_ly * Sensitivity;
    float target_vy = -raw_lx * Sensitivity;

    // 针对只有 0 和 99 的扳机键 (LT/RT)，直接算出期望的Z轴速度
    float target_vz = ((float)Current_PD.rt - (float)Current_PD.lt) * Z_limit * Sensitivity;

    // 针对只有 0 和 1 的肩键 (LB/RB)
    float target_w_gamma = ((float)Current_PD.btn_rb - (float)Current_PD.btn_lb) * Sensitivity;

    float target_w_alpha = raw_rx * Sensitivity * Euler_Limit;
    float target_w_beta  = raw_ry * Sensitivity * Euler_Limit;

    // 3. 核心魔法：一阶低通滤波 (平滑追赶)
    // SMOOTH_FACTOR 决定了柔顺程度，范围在 0.0 到 1.0 之间。
    // 值越小（如0.02），起步/刹车越柔顺（S型曲线越长），但手感越“肉”。
    // 值越大（如0.2），响应越快，但越生硬。100Hz下推荐 0.05 - 0.1。

    smooth_vx += SMOOTH_FACTOR * (target_vx - smooth_vx);
    smooth_vy += SMOOTH_FACTOR * (target_vy - smooth_vy);
    smooth_vz += SMOOTH_FACTOR * (target_vz - smooth_vz);

    smooth_w_alpha += SMOOTH_FACTOR * (target_w_alpha - smooth_w_alpha);
    smooth_w_beta  += SMOOTH_FACTOR * (target_w_beta  - smooth_w_beta);
    smooth_w_gamma += SMOOTH_FACTOR * (target_w_gamma - smooth_w_gamma);

    // 4. 将平滑后的速度积分，得出最终位姿
    Target_CP = Current_CP;

    switch (PL)
    {
    case None:
        Target_CP.x += smooth_vx * CONTROL_DT;
        Target_CP.y += smooth_vy * CONTROL_DT;
        Target_CP.z += smooth_vz * CONTROL_DT;

        Target_CP.alpha += smooth_w_alpha * CONTROL_DT;
        Target_CP.beta  += smooth_w_beta  * CONTROL_DT;
        Target_CP.gamma += smooth_w_gamma * CONTROL_DT;
        break;

    case Lock1:
        Target_CP.x += smooth_vx * CONTROL_DT;
        break;

    case Lock2:
        Target_CP.y += smooth_vy * CONTROL_DT;
        break;

    case Lock3:
        Target_CP.z += smooth_vz * CONTROL_DT;
        break;

    case Lock4:
        Target_CP.alpha += smooth_w_alpha * CONTROL_DT;
        break;

    case Lock5:
        Target_CP.beta += smooth_w_beta * CONTROL_DT;
        break;

    case Lock6:
        Target_CP.gamma += smooth_w_gamma * CONTROL_DT;
        break;

    default:
        break;
    }
}

void PadControl::Calculate_Target_Joint(const Pad_Params_t Current_PD, const Pad_Lock PL, const float Sensitivity)
{
    // 1. 摇杆死区消除 (Deadzone) - 极其重要！
    // 手柄回弹很难绝对回0，一般在 -5 到 5 之间波动，不加死区机器人会一直漂移
    const float DEADZONE = 10.0f;

    float raw_lx = (fabs(Current_PD.lx) > DEADZONE) ? Current_PD.lx : 0.0f;
    float raw_ly = (fabs(Current_PD.ly) > DEADZONE) ? Current_PD.ly : 0.0f;
    float raw_rx = (fabs(Current_PD.rx) > DEADZONE) ? Current_PD.rx : 0.0f;
    float raw_ry = (fabs(Current_PD.ry) > DEADZONE) ? Current_PD.ry : 0.0f;

    // 2. 计算当前的“期望速度” (Target Velocity)
    // 注意：这里只是算出了目标值，还没有加到坐标上
    float target_joint1 = -raw_lx * Sensitivity*Joint_Limit;
    float target_joint2 = raw_ly * Sensitivity*Joint_Limit;

    // 针对只有 0 和 99 的扳机键 (LT/RT)，直接算出期望的Z轴速度
    float target_joint3 = ((float)Current_PD.lt - (float)Current_PD.rt) * Joint_Limit * Sensitivity;

    // // 针对只有 0 和 1 的肩键 (LB/RB)
    float target_joint6 = ((float)Current_PD.btn_rb - (float)Current_PD.btn_lb) * Sensitivity*Joint_Limit*100;

    float target_joint4 = raw_rx * Sensitivity * Joint_Limit;
    float target_joint5  = raw_ry * Sensitivity * Joint_Limit*0.7;

    // 3. 核心魔法：一阶低通滤波 (平滑追赶)
    // SMOOTH_FACTOR 决定了柔顺程度，范围在 0.0 到 1.0 之间。
    // 值越小（如0.02），起步/刹车越柔顺（S型曲线越长），但手感越“肉”。
    // 值越大（如0.2），响应越快，但越生硬。100Hz下推荐 0.05 - 0.1。

    smooth_joint1 += SMOOTH_FACTOR * (target_joint1 - smooth_joint1);
    smooth_joint2 += SMOOTH_FACTOR * (target_joint2 - smooth_joint2);
    smooth_joint3 += SMOOTH_FACTOR * (target_joint3 - smooth_joint3);
    smooth_joint4 += SMOOTH_FACTOR * (target_joint4 - smooth_joint4);
    smooth_joint5 += SMOOTH_FACTOR * (target_joint5 - smooth_joint5);
    smooth_joint6 += SMOOTH_FACTOR * (target_joint6 - smooth_joint6);

    memset(JointRPM, 0, sizeof(float) * 6);

    switch (PL)
    {
    case None:
        JointRPM[0]=smooth_joint1;
        JointRPM[1]=smooth_joint2;
        JointRPM[2]=smooth_joint3;
        JointRPM[3]=smooth_joint4;
        JointRPM[4]=smooth_joint5;
        JointRPM[5]=smooth_joint6;
        break;

    case Lock1:
        JointRPM[0]=smooth_joint1;
        break;

    case Lock2:
        JointRPM[1]=smooth_joint2;
        break;

    case Lock3:
        JointRPM[2]=smooth_joint3;
        break;

    case Lock4:
        JointRPM[3]=smooth_joint4;
        break;

    case Lock5:
        JointRPM[4]=smooth_joint5;
        break;

    case Lock6:
        JointRPM[5]=smooth_joint6;
        break;

    default:
        break;
    }
}

void PadControl::Calculate_Target_ToolCP(const Pad_Params_t Current_PD, const Coordinates_Pose Current_CP, const float Sensitivity,Rotation_Matrix ZYZ_RM)
{
    float target_z=(float)Current_PD.dpad_y*Sensitivity;


    smooth_tool_z += SMOOTH_FACTOR * (target_z - smooth_tool_z);
    float v_world_x = ZYZ_RM.m[0][2] * smooth_tool_z;
    float v_world_y = ZYZ_RM.m[1][2] * smooth_tool_z;
    float v_world_z = ZYZ_RM.m[2][2] * smooth_tool_z;

    float target_w_gamma = ((float)Current_PD.btn_rb - (float)Current_PD.btn_lb) * Sensitivity;
    smooth_w_gamma += SMOOTH_FACTOR * (target_w_gamma - smooth_w_gamma);

    Target_CP=Current_CP;

    Target_CP.x += v_world_x * CONTROL_DT*60;
    Target_CP.y += v_world_y * CONTROL_DT*60;
    Target_CP.z += v_world_z * CONTROL_DT*60;

    Target_CP.gamma += smooth_w_gamma * CONTROL_DT;

}

////简单限幅
// void PadControl::Calculate_Deta_CP(const Pad_Params_t Current_PD, const Coordinates_Pose Current_CP)
// {
//
//     auto applyRateLimit = [](int8_t current, int8_t last, uint8_t limit) -> int8_t
//     {
//         int16_t diff = current - last; // 用 int16_t 防止 int8_t 溢出
//         if (diff > limit)  return last + limit;
//         if (diff < -limit) return last - limit;
//         return current;
//     };
//
//     Target_PD.lx=applyRateLimit(Current_PD.lx,Target_PD.lx,Limit);
//     Target_PD.ly=applyRateLimit(Current_PD.ly,Target_PD.ly,Limit);
//     Target_PD.rt=Current_PD.rt*Z_limit;
//     Target_PD.lt=Current_PD.lt*Z_limit;
//     Target_PD.rx=applyRateLimit(Current_PD.rx,Target_PD.rx,Limit);
//     Target_PD.ry=applyRateLimit(Current_PD.ry,Target_PD.ry,Limit);
//     Target_PD.btn_lb=Current_PD.btn_lb;
//     Target_PD.btn_rb=Current_PD.btn_rb;
//
//
//     Last_PD = Target_PD;
//
//     Deta_CP = Current_CP;
//     Deta_CP.x = -(float)Target_PD.ly * Sensitivity * CONTROL_DT + Current_CP.x;
//     Deta_CP.y = -(float)Target_PD.lx * Sensitivity * CONTROL_DT + Current_CP.y;
//     Deta_CP.z = (float)Target_PD.rt * Sensitivity * CONTROL_DT -(float)Target_PD.lt * Sensitivity * CONTROL_DT+Current_CP.z;
//
//     Deta_CP.alpha=(float)Target_PD.rx*Sensitivity * CONTROL_DT*0.01f + Current_CP.alpha;
//     Deta_CP.beta=(float)Target_PD.ry*Sensitivity * CONTROL_DT*0.01f + Current_CP.beta;
//     Deta_CP.gamma=(float)Target_PD.btn_rb * Sensitivity * CONTROL_DT -(float)Target_PD.btn_lb * Sensitivity * CONTROL_DT+Current_CP.gamma;
//
// }

void PadControl::Control_Once(const float* current_angle_rad, const float* Best_Angle_Rad)
{
    for (int i=0;i<6;i++)
    {
        JointRPM[i]=(Best_Angle_Rad[i]-current_angle_rad[i])/CONTROL_DT*RAD_TO_RPM;
    }
}


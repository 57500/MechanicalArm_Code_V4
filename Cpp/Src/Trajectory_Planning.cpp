//
// Created by 57500 on 2026/3/3.
//

#include "Trajectory_Planning.h"

#include <cstring>

#include "can.h"
#include "Inverse_Kinematics.h"
#include "MotorControl.h"
#include "usart.h"

/**
 * @brief 轨迹规划的构造函数
 * @param NULL
 * @return NULL
 */
Trajectory_Planning::Trajectory_Planning()
{
    Clear();
}

/**
 * @brief 重置所有过程量
 * @param NULL
 * @return NULL
 */
void Trajectory_Planning::Clear(void)
{
    is_first_run=1;
    Step = 0;
    Count=0;
    memset(JointRPM,0,sizeof(JointRPM));
    memset(Target_Angle_Rad,0,sizeof(Target_Angle_Rad));
    memset(Current_Angle_Rad,0,sizeof(Current_Angle_Rad));
    memset(CP_Ref,0,sizeof(CP_Ref));
    memset(V_Ref,0,sizeof(V_Ref));

}

/**
 * @brief 依据路程比例K计算当前位姿与目标位姿之间对应的插值位姿
 * @param Target_CP：目标位姿
 * @param Current_CP：当前位姿
 * @param K：路程比例
 * @return 返回插值位姿
 */
Coordinates_Pose Trajectory_Planning::Interpolation(Coordinates_Pose Target_CP, Coordinates_Pose Current_CP, float K)
{
    if(K > 1.0f) K = 1.0f;
    if(K < 0.0f) K = 0.0f;

    Coordinates_Pose Middle_CP;

    Middle_CP.x = Current_CP.x + K * (Target_CP.x - Current_CP.x);
    Middle_CP.y = Current_CP.y + K * (Target_CP.y - Current_CP.y);
    Middle_CP.z = Current_CP.z + K * (Target_CP.z - Current_CP.z);

    // 计算最短角度差，结果在 [-π, π] 区间
    auto shortest_angle_diff = [](float target, float current) -> float {
        float diff = target - current;
        while (diff > PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;
        return diff;
    };

    // 对每个欧拉角进行沿最短路径的插值
    float delta_alpha = shortest_angle_diff(Target_CP.alpha, Current_CP.alpha);
    float delta_beta  = shortest_angle_diff(Target_CP.beta,  Current_CP.beta);
    float delta_gamma = shortest_angle_diff(Target_CP.gamma, Current_CP.gamma);

    Middle_CP.alpha = Current_CP.alpha + K * delta_alpha;
    Middle_CP.beta  = Current_CP.beta  + K * delta_beta;
    Middle_CP.gamma = Current_CP.gamma + K * delta_gamma;

    return Middle_CP;
}

/**
 * @brief 在计算S型曲线速度中规划直线轨迹
 * @param Target_CP：目标位姿
 * @param Current_CP：当前位姿
 * @return NULL
 */
void Trajectory_Planning::S_Curve_Profile(Coordinates_Pose Target_CP, Coordinates_Pose Current_CP)
{
    Clear();
    // 计算总距离 S
    float dx = Target_CP.x - Current_CP.x;
    float dy = Target_CP.y - Current_CP.y;
    float dz = Target_CP.z - Current_CP.z;
    float S = sqrtf(dx*dx + dy*dy + dz*dz);

    if (S < 1e-6f) Step=0;

    // 规划参数
    float Omg = OMG_MAX;
    float Vmax = 0.0f, T1 = 0.0f, T0 = 0.0f, T = 0.0f;

    float half_Vmax_over_Omg = 0.0f; // Vmax/(2*Omg)

    if (S >= S_ACC_MAX)
    {
        // 长距离模式：三段式
        Vmax = V_MAX;
        T1 = T1_LIMIT;
        T0 = (S - S_ACC_MAX) / Vmax;
        T = T0 + T1;
        half_Vmax_over_Omg = Vmax / (2.0f * Omg);
    }
    else
    {
        // 短距离模式：两段式
        Vmax = S * Omg / PI;
        T1 = T1_LIMIT;
        T0 = 0.0f;
        T = T1;
        half_Vmax_over_Omg = Vmax / (2.0f * Omg);
    }

    Step = (uint16_t)(T / CONTROL_DT + 0.5f);
    if (Step == 0) Step = 1;

    // 生成轨迹点
    for (int i = 0; i < Step; ++i)
    {
        float t = (i + 1) * CONTROL_DT;

        float s = 0.0f; // 当前位移
        float v = 0.0f; // 当前速度

        // 长距离模式
        if (S >= S_ACC_MAX)
        {
            // 加速段
            if (t <= T1/2.0f)
            {
                v = (Vmax/2.0f) * sinf(Omg * t - PI/2) + Vmax/2.0f;
                s = Vmax * t / 2.0f - half_Vmax_over_Omg * cosf(Omg * t - PI/2);
            }
            // 匀速段
            else if (t > T1/2.0f && t <= T1/2.0f + T0)
            {
                v = Vmax;
                s = S_ACC_MAX/2.0f + Vmax * (t - T1/2.0f);
            }
            // 减速段
            else
            {
                float t_dec = t - T0;
                v = (Vmax/2.0f) * sinf(Omg * t_dec - PI/2) + Vmax/2.0f;
                s = Vmax * t_dec / 2.0f - half_Vmax_over_Omg * cosf(Omg * t_dec - PI/2) + Vmax * T0;
            }
        }
        // 短距离模式
        else
        {
            v = (Vmax/2.0f) * sinf(Omg * t - PI/2) + Vmax/2.0f;
            s = Vmax * t / 2.0f - half_Vmax_over_Omg * cosf(Omg * t - PI/2);
        }

        // 确保位移不超过总距离（应对所有阶段的浮点误差）
        if (s > S) s = S;
        if (s < 0.0f) s = 0.0f;

        V_Ref[i] = v;
        CP_Ref[i] = Interpolation(Target_CP, Current_CP, s / S);
    }
}

/**
 * @brief 更新当前关节角度
 * @param current_angle_rad：当前关节角度
 * @return NULL
 */
void Trajectory_Planning::Update_Current_Angle_Rad(const float* current_angle_rad)
{
    for (int i = 0; i < 6; i++)
    {
        Current_Angle_Rad[i] = current_angle_rad[i];
    }
}

/**
 * @brief 更新期望关节角度
 * @param Best_Angle_Rad：期望角度
 * @return NULL
 */
void Trajectory_Planning::Update_Target_Angle_Rad(const float* Best_Angle_Rad)
{
    for (int i = 0; i < 6; i++)
    {
        if (is_first_run==1)
        {
            Last_Target_Angle_Rad[i]=Best_Angle_Rad[i];
            Target_Angle_Rad[i]=Best_Angle_Rad[i];
            is_first_run=0;
        }
        else
        {
            Last_Target_Angle_Rad[i]=Target_Angle_Rad[i];
            Target_Angle_Rad[i]=Best_Angle_Rad[i];
        }
    }
}

/**
 * @brief 位置环计算关节转速
 * @param NULL
 * @return NULL
 */
void Trajectory_Planning::Calculate_Current_JointRPM(void)
{
    if (Count <= 2)
    {
        for (int i = 0; i < 6; i++) JointRPM[i] = 0.000f;

        return;
    }

    for (int i = 0; i < 6; i++)
    {
        float Feedforward_Velocity=(Target_Angle_Rad[i]-Last_Target_Angle_Rad[i])/CONTROL_DT;
        float Error=Target_Angle_Rad[i]-Current_Angle_Rad[i];
        float P_Out=PID_Kp[i]*Error;
        float Total_Vel_Rad=Feedforward_Velocity+P_Out;
        JointRPM[i]=Total_Vel_Rad*RAD_TO_RPM;
    }
}

/**
 * @brief PTP模式单次控制
 * @param current_angle_rad：当前关节角度
 * @param Best_Angle_Rad：期望关节角度
 * @return NULL
 */
void Trajectory_Planning::Control_Once(const float* current_angle_rad,const float* Best_Angle_Rad)
{
    Count++;
    Step--;

    Update_Current_Angle_Rad(current_angle_rad);
    Update_Target_Angle_Rad(Best_Angle_Rad);
    Calculate_Current_JointRPM();
}
//
// Created by 57500 on 2026/3/22.
//

#ifndef MECHANICALARM_CODE_V4_PADCONTROL_H
#define MECHANICALARM_CODE_V4_PADCONTROL_H
#include "RobotTypes.h"

class PadControl
{
public:
    PadControl();

    void Calculate_Target_CP(const Pad_Params_t Current_PD,const Coordinates_Pose Current_CP,const Pad_Lock PL,const float Sensitivity);

    void Calculate_Target_Joint(const Pad_Params_t Current_PD,const Pad_Lock PL,const float Sensitivity);

    void Control_Once(const float* current_angle_rad,const float* Best_Angle_Rad);

    void Clear();

    Coordinates_Pose Target_CP;

    float JointRPM[6];

private:

    // 在 PadControl.h 中添加：
    float smooth_vx = 0.0f;
    float smooth_vy = 0.0f;
    float smooth_vz = 0.0f;
    float smooth_w_alpha = 0.0f;
    float smooth_w_beta = 0.0f;
    float smooth_w_gamma = 0.0f;

    float smooth_joint1 = 0.0f;
    float smooth_joint2 = 0.0f;
    float smooth_joint3 = 0.0f;
    float smooth_joint4 = 0.0f;
    float smooth_joint5 = 0.0f;
    float smooth_joint6 = 0.0f;


    static constexpr float CONTROL_DT=0.01f;
    // static constexpr uint8_t Limit=50;
    static constexpr float Z_limit=0.6;
    static constexpr float Euler_Limit=0.01f;
    static constexpr float Joint_Limit=0.1f;
    static constexpr float RAD_TO_RPM=9.549297f;
    static constexpr float SMOOTH_FACTOR = 0.05f;

    Pad_Params_t Last_PD;
    Pad_Params_t Target_PD;
};

#endif //MECHANICALARM_CODE_V4_PADCONTROL_H
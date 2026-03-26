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

    void Calculate_Deta_CP(const Pad_Params_t Current_PD,const Coordinates_Pose Current_CP);

    void Control_Once(const float* current_angle_rad,const float* Best_Angle_Rad);

    void Clear();

    Coordinates_Pose Deta_CP;

    float JointRPM[6];

private:

    static constexpr float CONTROL_DT=0.01f;
    static constexpr float Sensitivity=0.8f;
    static constexpr uint8_t Limit=20;
    static constexpr float Z_limit=0.6;
    static constexpr float RAD_TO_RPM=9.549297f;

    Pad_Params_t Last_PD;
    Pad_Params_t Target_PD;
};

#endif //MECHANICALARM_CODE_V4_PADCONTROL_H
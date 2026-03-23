//
// Created by 57500 on 2026/3/22.
//

#include "PadControl.h"

#include <cmath>
#include <cstring>

PadControl::PadControl()
{
    memset(JointRPM,0,sizeof(float)*6);
    Deta_CP={};
    Target_PD = {};
    Last_PD={};
}

void PadControl::Clear()
{
    memset(JointRPM,0,sizeof(float)*6);
    Deta_CP={};
    Target_PD = {};
    Last_PD={};
}

void PadControl::Calculate_Deta_CP(const Pad_Params_t Current_PD, const Coordinates_Pose Current_CP)
{
    float diff = Current_PD.lx - Last_PD.lx;

    if (diff > Limit)
    {
        Target_PD.lx = Last_PD.lx + Limit;
    }
    else if (diff < -Limit)
    {
        Target_PD.lx = Last_PD.lx - Limit;
    }
    else
    {
        Target_PD.lx = Current_PD.lx;
    }

    Last_PD = Target_PD;

    Deta_CP = Current_CP;
    Deta_CP.x = (float)Target_PD.lx * Sensitivity * CONTROL_DT + Current_CP.x;
}

void PadControl::Control_Once(const float* current_angle_rad, const float* Best_Angle_Rad)
{
    for (int i=0;i<6;i++)
    {
        JointRPM[i]=(Best_Angle_Rad[i]-current_angle_rad[i])/CONTROL_DT*RAD_TO_RPM;
    }
}


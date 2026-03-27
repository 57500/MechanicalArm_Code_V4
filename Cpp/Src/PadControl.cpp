//
// Created by 57500 on 2026/3/22.
//

#include "PadControl.h"

#include <cmath>
#include <cstring>

#include "usart.h"

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

    auto applyRateLimit = [](int8_t current, int8_t last, uint8_t limit) -> int8_t
    {
        int16_t diff = current - last; // 用 int16_t 防止 int8_t 溢出
        if (diff > limit)  return last + limit;
        if (diff < -limit) return last - limit;
        return current;
    };

    Target_PD.lx=applyRateLimit(Current_PD.lx,Target_PD.lx,Limit);
    Target_PD.ly=applyRateLimit(Current_PD.ly,Target_PD.ly,Limit);
    Target_PD.rt=Current_PD.rt*Z_limit;
    Target_PD.lt=Current_PD.lt*Z_limit;
    Target_PD.rx=applyRateLimit(Current_PD.rx,Target_PD.rx,Limit);
    Target_PD.ry=applyRateLimit(Current_PD.ry,Target_PD.ry,Limit);
    Target_PD.btn_lb=Current_PD.btn_lb;
    Target_PD.btn_rb=Current_PD.btn_rb;
    

    Last_PD = Target_PD;

    Deta_CP = Current_CP;
    Deta_CP.x = -(float)Target_PD.ly * Sensitivity * CONTROL_DT + Current_CP.x;
    Deta_CP.y = -(float)Target_PD.lx * Sensitivity * CONTROL_DT + Current_CP.y;
    Deta_CP.z = (float)Target_PD.rt * Sensitivity * CONTROL_DT -(float)Target_PD.lt * Sensitivity * CONTROL_DT+Current_CP.z;

    Deta_CP.alpha=(float)Target_PD.rx*Sensitivity * CONTROL_DT*0.01f + Current_CP.alpha;
    Deta_CP.beta=(float)Target_PD.ry*Sensitivity * CONTROL_DT*0.01f + Current_CP.beta;
    Deta_CP.gamma=(float)Target_PD.btn_rb * Sensitivity * CONTROL_DT -(float)Target_PD.btn_lb * Sensitivity * CONTROL_DT+Current_CP.gamma;



}

void PadControl::Control_Once(const float* current_angle_rad, const float* Best_Angle_Rad)
{
    for (int i=0;i<6;i++)
    {
        JointRPM[i]=(Best_Angle_Rad[i]-current_angle_rad[i])/CONTROL_DT*RAD_TO_RPM;
    }
}


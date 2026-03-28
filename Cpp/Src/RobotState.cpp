//
// Created by 57500 on 2026/3/2.
//
#include "RobotState.h"

#include <cstdio>
#include <cstring>

#include "can.h"
#include "MotorControl.h"
#include "usart.h"


RobotState::RobotState()
{
    Clear();
}

void RobotState::Clear()
{
    memset(Current_Angle_Rad,0,sizeof(Current_Angle_Rad));
    memset(Next_Best_Angle_Rad,0,sizeof(Next_Best_Angle_Rad));
    Current_CP={0};
    Target_CP={0};
    Current_Pad={0};
    Last_Pad={0};
    Current_Pad_Mode=Standby;
    Pad_Cartesian_Sensitivity=0.8f;
    Current_Pad_Lock=None;
}



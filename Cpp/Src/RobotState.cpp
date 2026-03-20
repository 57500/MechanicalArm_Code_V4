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
    memset(Current_Angle_Rad,0,sizeof(Current_Angle_Rad));
    memset(Next_Best_Angle_Rad,0,sizeof(Next_Best_Angle_Rad));
    Current_CP={0};
    Target_CP={0};
}

void RobotState::Clear()
{
    memset(Current_Angle_Rad,0,sizeof(Current_Angle_Rad));
    memset(Next_Best_Angle_Rad,0,sizeof(Next_Best_Angle_Rad));
    Current_CP={0};
    Target_CP={0};
}



//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_ROBOTSTATE_H
#define MECHANICALARM_CODE_V2_ROBOTSTATE_H

#include "RobotTypes.h"

class RobotState
{
public:
    RobotState();

    void Clear(void);

    float Current_Angle_Rad[6];

    float Next_Best_Angle_Rad[6];

    Coordinates_Pose Current_CP;

    Coordinates_Pose Target_CP;

    Pad_Params_t Current_Pad;

    Pad_Params_t Last_Pad;

    Pad_Mode Current_Pad_Mode;

    float Pad_Cartesian_Sensitivity;

    Pad_Lock Current_Pad_Lock;

    bool Pad_Home;

    bool is_first_home;

};

#endif //MECHANICALARM_CODE_V2_ROBOTSTATE_H
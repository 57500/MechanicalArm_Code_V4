//
// Created by 57500 on 2026/3/3.
//

#ifndef MECHANICALARM_CODE_V2_TRAJECTORY_PLANNING_H
#define MECHANICALARM_CODE_V2_TRAJECTORY_PLANNING_H

#include <cstdint>

#include "RobotTypes.h"
#include "cmath"

class Trajectory_Planning
{
public:
    Trajectory_Planning();

    void S_Curve_Profile(Coordinates_Pose Target_CP,
        Coordinates_Pose Current_CP);

    void Control_Once(const float* current_angle_rad,const float* Best_Angle_Rad);

    void Clear(void);

    uint32_t Step;

    uint32_t Count;

    Coordinates_Pose CP_Ref[1024];

    float JointRPM[6];

private:
    Coordinates_Pose Interpolation(Coordinates_Pose Target_CP, Coordinates_Pose Current_CP, float K);

    void Update_Current_Angle_Rad(const float* current_angle_rad);

    void Update_Target_Angle_Rad(const float* Best_Angle_Rad);

    void Calculate_Current_JointRPM(void);

    static constexpr float A_MAX=500.0f;
    static constexpr float V_MAX=300.0f;
    static constexpr float OMG_MAX=(2.0f * A_MAX / V_MAX);
    static constexpr float S_ACC_MAX=(PI * V_MAX / OMG_MAX);
    static constexpr float T1_LIMIT=(2.0f * PI / OMG_MAX);
    static constexpr float CONTROL_DT=0.01f;
    static constexpr float RAD_TO_RPM=9.549297f;

    static constexpr float PID_Kp[6]={0.65,0.5,0.75,1,2,10};
    uint8_t is_first_run;
    float Target_Angle_Rad[6];
    float Last_Target_Angle_Rad[6];
    float Current_Angle_Rad[6];

    float V_Ref[1024];


};

#endif //MECHANICALARM_CODE_V2_TRAJECTORY_PLANNING_H
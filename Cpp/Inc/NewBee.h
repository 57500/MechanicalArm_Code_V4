//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_NEWBEE_H
#define MECHANICALARM_CODE_V2_NEWBEE_H

#include "RobotTypes.h"
#include "Forward_Kinematics.h"
#include "Inverse_Kinematics.h"
#include "MotorControl.h"
#include "RobotState.h"
#include "Trajectory_Planning.h"

class NewBee
{
public:
    NewBee();

    void Clear(void);

    void UpDate(void);

    void Control_All_Motor(const float* rpm);
    void Emergency_Stop(void);

    float* Get_Current_Angle_Rad(void);
    uint32_t Get_Current_Step(void);

    void UpDate_S_Curve_Profile(void);
    void UpDate_Target_CP(const Coordinates_Pose& target_cp);
    void UpDate_Current_CP(void);
    void UpDate_Current_Angle_Rad(void);

private:
    ForwardKinematics fk;
    Inverse_Kinematics ik;

    Trajectory_Planning tp;

    MotorControl motor1;
    MotorControl motor2;
    MotorControl motor3;
    MotorControl motor4;
    MotorControl motor5;
    MotorControl motor6;

    RobotState state;
};

#endif //MECHANICALARM_CODE_V2_NEWBEE_H
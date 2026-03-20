//
// Created by 57500 on 2026/2/28.
//

#ifndef MECHANICALARM_CODE_V2_MOTORCONTROL_H
#define MECHANICALARM_CODE_V2_MOTORCONTROL_H
#include <sys/_stdint.h>
#include "RobotTypes.h"



class MotorControl
{
public:
    MotorControl(uint8_t Addr);

    void Motor_Enable(void);

    void Motor_VelControl(float Speed,uint8_t Acc,uint8_t Syn);

    void Motor_PosControl(uint16_t Vel,
    uint8_t Acc,float Rad,uint8_t RaF,uint8_t Syn);

    void Read_Motor_Param(uint8_t Param);

    static void Motor_SynMode(void);

    void Change_MotorAddr(uint8_t Change_Addr);

    static void Read_Motor_Pos(uint8_t addr);

private:
    uint8_t Addr;

    static constexpr float RATIOS[7] = {0.0f, 50.0f, 51.0f, 51.0f, 51.0f, 27.0f, 51.0f};
};

#endif //MECHANICALARM_CODE_V2_MOTORCONTROL_H
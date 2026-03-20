//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_FORWARD_KINEMATICS_H
#define MECHANICALARM_CODE_V2_FORWARD_KINEMATICS_H

#include "RobotTypes.h"

class ForwardKinematics
{
public:
    ForwardKinematics();

    Coordinates_Pose Forward_Kinematics(const float* JointAngles);

private:
    typedef struct
    {
        float m[4][4];
    }Homogeneous_Matrix;

    Homogeneous_Matrix Identity_Matrix(void);

    Homogeneous_Matrix TransferMatrix_Multiply(const Homogeneous_Matrix& A,const Homogeneous_Matrix& B);

    Homogeneous_Matrix DH_Transform(float alpha,float a,float d,float theta);


};

#endif //MECHANICALARM_CODE_V2_FORWARD_KINEMATICS_H
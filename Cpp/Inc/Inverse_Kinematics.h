//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_INVERSE_KINEMATICS_H
#define MECHANICALARM_CODE_V2_INVERSE_KINEMATICS_H
#include "RobotTypes.h"
#include <cmath>
#include <cstdint>

class Inverse_Kinematics
{
public:
    Inverse_Kinematics();

    void Clear(void);

    bool Solve_FinalTheta(const Coordinates_Pose& target_pose,const float current_angles[6],float best_angles[6],const float CurrentTheta4);

private:
    typedef struct
    {
        float m[3][3];
    }Rotation_Matrix;

    void Wrist_Position_Optimized(const Coordinates_Pose& Coor_Pos);

    bool Solve_Trig(float a, float b, float c, float* sol);

    void Solve_Theta3(void);

    void Solve_Theta2(void);

    void Solve_Theta1(void);

    Rotation_Matrix RotationMatrix_Multiply(const Rotation_Matrix& A, const Rotation_Matrix& B);

    Rotation_Matrix RotationMatrix_Transpose(const Rotation_Matrix& R);

    Rotation_Matrix Compute_Link_R(uint8_t m, float theta);

    Rotation_Matrix ZYZ_to_RotationMatrix(const Coordinates_Pose& Coor_Pos);

    void Solve_Theta456(const Coordinates_Pose& Coor_Pos,const float CurrentTheta4);

    bool SelectBestSolution(const float current_angles[6],float best_angles[6]);

    static constexpr float End_WristVector[3]={0.0f,0.0f,-130.0f};

    float Wrist_Vector[3];

    float Theta3[4];

    float Theta2[4];

    float Theta1[4];

    float Theta4[4];

    float Theta5[4];

    float Theta6[4];

    float State[4];

    static constexpr float DEFAULT_WEIGHTS[6] = {20.0f, 15.0f, 12.0f, 5.0f, 3.0f, 1.0f};

    int last_best_index = -1; // 记录上一帧选中的解索引

    static constexpr float SWITCH_PENALTY = 10.0f;// 切换分支的惩罚系数，>1 表示不鼓励切换``
};

#endif //MECHANICALARM_CODE_V2_INVERSE_KINEMATICS_H
//
// Created by 57500 on 2026/3/2.
//

#include "Forward_Kinematics.h"
#include <cmath>
#include <cstdint>
#include <cstring>

/**
 * @brief 正向运动学类的默认构造函数
 * @param NULL
 * @return NULL
 */
ForwardKinematics::ForwardKinematics(void)
{

}

/**
 * @brief 创建单位变换矩阵
 * @param NULL
 * @return 单位变换矩阵
 */
ForwardKinematics::Homogeneous_Matrix ForwardKinematics::Identity_Matrix(void)
{
    Homogeneous_Matrix I;
    memset(&I,0,sizeof(I));
    for(int i=0;i<4;i++)
    {
        I.m[i][i]=1.0f;
    }
    return I;
}

/**
 * @brief 计算两个齐次变换矩阵的乘积
 * @param A 第一个4x4齐次变换矩阵
 * @param B 第二个4x4齐次变换矩阵
 * @return 矩阵乘积 C = A × B
 */
ForwardKinematics::Homogeneous_Matrix ForwardKinematics::TransferMatrix_Multiply(const Homogeneous_Matrix& A,const Homogeneous_Matrix& B)
{
    Homogeneous_Matrix C;
    memset(&C,0,sizeof(C));
    for (uint8_t i=0;i<4;i++)
    {
        for (uint8_t j=0;j<4;j++)
        {
            for (uint8_t k=0;k<4;k++)
            {
                C.m[i][j]+=A.m[i][k]*B.m[k][j];
            }
        }
    }
    return C;
}

/**
 * @brief 根据Denavit-Hartenberg (DH) 参数计算单个关节的齐次变换矩阵
 * @param alpha 连杆扭角 (rad)
 * @param a 连杆长度 (mm)
 * @param d 连杆偏距 (mm)
 * @param theta 关节角度 (rad)
 * @return 4x4 DH变换矩阵
 */
ForwardKinematics::Homogeneous_Matrix ForwardKinematics::DH_Transform(float alpha,float a,float d,float theta)
{
    Homogeneous_Matrix T;
    memset(&T,0,sizeof(T));
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(alpha);
    float sa = sinf(alpha);

    T.m[0][0] = ct;        T.m[0][1] = -st;       T.m[0][2] = 0;         T.m[0][3] = a;
    T.m[1][0] = st*ca;     T.m[1][1] = ct*ca;     T.m[1][2] = -sa;       T.m[1][3] = -d*sa;
    T.m[2][0] = st*sa;     T.m[2][1] = ct*sa;     T.m[2][2] = ca;        T.m[2][3] = d*ca;
    T.m[3][0] = 0;         T.m[3][1] = 0;         T.m[3][2] = 0;         T.m[3][3] = 1;

    return T;
}

/**
 * @brief 计算机械臂的正向运动学，根据关节角度计算末端执行器的位姿
 * @param JointAngles 指向包含6个关节角度(rad)的数组指针
 * @return 末端执行器的位姿（位置XYZ和ZYZ欧拉角）
 * @note 如果输入指针为空，返回零位姿
 */
Coordinates_Pose ForwardKinematics::Forward_Kinematics(const float* JointAngles)
{
    if (JointAngles == nullptr)
    {
        return Coordinates_Pose{0,0,0,0,0,0};
    }

    Coordinates_Pose CP;
    Homogeneous_Matrix X = Identity_Matrix();
    for (uint8_t i = 0; i < 6; i++)
    {
        Homogeneous_Matrix Y;
        Y= DH_Transform(DH_Params[i].alpha, DH_Params[i].a,
            DH_Params[i].d, JointAngles[i]);
        X = TransferMatrix_Multiply(X,Y);
    }


    CP.x = X.m[0][3];
    CP.y = X.m[1][3];
    CP.z = X.m[2][3];

    // ZYZ 欧拉角计算
    if (fabsf(X.m[2][2]) < 1.0f - 1e-6f)
    {
        // 一般情况：β ≠ 0 且 β ≠ π
        CP.alpha = atan2f(X.m[1][2], X.m[0][2]);        // 第一个 Z 旋转 (α)
        CP.beta = atan2f(sqrtf(X.m[0][2]*X.m[0][2] + X.m[1][2]*X.m[1][2]), X.m[2][2]);  // Y 旋转 (β)
        CP.gamma = atan2f(X.m[2][1], -X.m[2][0]);      // 第二个 Z 旋转 (γ)
    }
    else
    {
        // 奇异情况：β = 0 或 β = π
        CP.alpha = 0.0f;
        if (X.m[2][2] > 0) {  // β = 0
            CP.beta = 0.0f;
            CP.gamma = atan2f(-X.m[1][0], X.m[0][0]);  // α + γ
        } else {  // β = π
            CP.beta = PI;
            CP.gamma = atan2f(X.m[1][0], -X.m[0][0]);  // γ - α
        }
    }

    return CP;
}
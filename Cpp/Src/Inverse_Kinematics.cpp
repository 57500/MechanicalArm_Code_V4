//
// Created by 57500 on 2026/3/2.
//

#include "Inverse_Kinematics.h"

#include <cstdint>
#include <cstring>

#include "usart.h"

/**
 * @brief 逆运动学类的构造函数
 * @param NULL
 * @return NULL
 */
Inverse_Kinematics::Inverse_Kinematics()
{
    Clear();
}

/**
 * @brief 清除所有内部状态和计算结果
 * @param NULL
 * @return NULL
 */
void Inverse_Kinematics::Clear(void)
{
    memset(Wrist_Vector, 0, sizeof(Wrist_Vector));
    memset(Theta3, 0, sizeof(Theta3));
    memset(Theta2, 0, sizeof(Theta2));
    memset(Theta1, 0, sizeof(Theta1));
    memset(Theta4, 0, sizeof(Theta4));
    memset(Theta5, 0, sizeof(Theta5));
    memset(Theta6, 0, sizeof(Theta6));
    memset(State, 0, sizeof(State));
}

/**
 * @brief 根据目标末端位姿计算腕点坐标
 * @param Coor_Pos 目标末端位姿（位置XYZ + ZYZ欧拉角）
 * @return NULL
 */
void Inverse_Kinematics::Wrist_Position_Optimized(const Coordinates_Pose& Coor_Pos)
{
    float alpha = Coor_Pos.alpha;
    float beta  = Coor_Pos.beta;
    // float gamma = Coor_Pos[5];

    // 预计算三角函数
    float ca = cosf(alpha); float sa = sinf(alpha);
    float cb = cosf(beta);  float sb = sinf(beta);
    // float cg = cosf(gamma); float sg = sinf(gamma);

    float R02 = ca * sb;  // R[0][2]
    float R12 = sa * sb;  // R[1][2]
    float R22 = cb;       // R[2][2]

    // 计算腕点坐标
    Wrist_Vector[0] = Coor_Pos.x + End_WristVector[2] * R02;  // x - 130 * R[0][2]
    Wrist_Vector[1] = Coor_Pos.y + End_WristVector[2] * R12;  // y - 130 * R[1][2]
    Wrist_Vector[2] = Coor_Pos.z + End_WristVector[2] * R22;  // z - 130 * R[2][2]
}

/**
 * @brief 求解三角方程 a*cos(x) + b*sin(x) = c
 * @param a 方程中 cos(x) 的系数
 * @param b 方程中 sin(x) 的系数
 * @param c 方程右侧的常数
 * @param sol 用于存储解的输出数组，大小为2。sol[0]和sol[1]将存储两个可能的解（如果存在）。
 *            解的范围在 [-π, π] 区间内。如果方程有唯一解，两个值可能相同。
 * @return bool 如果方程至少有一个实数解，返回 true；否则返回 false，且 sol 内容未定义。
 */
bool Inverse_Kinematics::Solve_Trig(float a, float b, float c, float* sol)
{
    const float EPS = 1e-6f; // 判定为零的阈值

    // 计算辅助角公式中的 R 和 phi: R * cos(x - phi) = c
    float R_sq = a * a + b * b;
    float R = sqrtf(R_sq);

    // 情况1: R 近似为 0，即 a 和 b 都接近 0
    if (R < EPS)
    {
        // 原方程退化为 0 = c
        if (fabsf(c) < EPS)
        {
            // 0=0 恒成立，有无穷多解。此处返回两个常用的基础解。
            sol[0] = 0.0f;
            sol[1] = PI;
            return true;
        }
        else
        {
            // 0 = c (c不为零)，无解
            return false;
        }
    }

    // 计算比值，并检查是否在余弦函数的定义域 [-1, 1] 内
    float ratio = c / R;
    if (ratio < -1.0f - EPS || ratio > 1.0f + EPS)
    {
        // |c/R| > 1，无实数解
        return false;
    }

    // 将 ratio 钳制到 [-1, 1] 区间内，以处理因浮点误差导致的轻微越界
    if (ratio < -1.0f) ratio = -1.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 计算辅助角 phi (atan2 确保象限正确)
    float phi = atan2f(b, a); // 注意：atan2(y, x) 对应 atan2(b, a)

    // 计算 acos(ratio) 得到基础角度差 delta
    float delta = acosf(ratio);

    // 方程的通解为: x = phi ± delta + 2kπ, k ∈ Z
    // 这里我们取主值，即 k=0 的情况，得到两个位于 [-π, π] 或附近的基本解
    sol[0] = phi + delta;   // 解1
    sol[1] = phi - delta;   // 解2

    return true;
}

/**
 * @brief 求解第三关节角 θ₃
 * @param NULL
 * @return NULL
 * @note 根据腕点坐标和DH参数建立三角方程，调用 Solve_Trig 求解
 */
void Inverse_Kinematics::Solve_Theta3(void)
{
    float R;

    R = Wrist_Vector[0]*Wrist_Vector[0] +
        Wrist_Vector[1]*Wrist_Vector[1] +
        (Wrist_Vector[2]-161)*(Wrist_Vector[2]-161);

    // 对于 a₁=0 的情况
    float a = 19052.0f;      // 400*a₃ = 400 * 47.63
    float b = -74000.0f;     // -400*d₄ = -400 * 185
    float c = R - 76493.6169f; // R - (a₃² + d₄² + a₂²)

    float theta3[2]={0,0};
    if (Solve_Trig(a,b,c,theta3)==false)
    {
        State[0]=0;
        State[1]=0;
        State[2]=0;
        State[3]=0;
    }
    else if (theta3[0]==theta3[1])
    {
        State[0]=1;
        State[1]=1;
        State[2]=0;
        State[3]=0;
    }
    else
    {
        State[0]=1;
        State[1]=1;
        State[2]=1;
        State[3]=1;
    }

    Theta3[0] = theta3[0];
    Theta3[1] = theta3[0];
    Theta3[2] = theta3[1];
    Theta3[3] = theta3[1];
}

/**
 * @brief 求解第二关节角 θ₂
 * @param NULL
 * @return NULL
 * @note 为每个有效的 θ₃ 解计算对应的 θ₂
 */
void Inverse_Kinematics::Solve_Theta2(void)
{
    float f1, f2;
    float a_coeff, b_coeff, c_const;
    float z_target = Wrist_Vector[2] - 161.0f;

    if (State[0]==1&&State[1]==1)
    {
        f1 = 47.63f * cosf(Theta3[0]) - 185.0f * sinf(Theta3[0]) + 200.0f;
        f2 = 47.63f * sinf(Theta3[0]) + 185.0f * cosf(Theta3[0]);

        a_coeff = -f2;
        b_coeff = -f1;
        c_const = z_target;

        float theta2[2]={0,0};
        if (Solve_Trig(a_coeff, b_coeff, c_const, theta2)==false)
        {
            State[0]=0;
            State[1]=0;
        }
        else if (theta2[0]==theta2[1])
        {
            State[1]=0;
        }
        Theta2[0] = theta2[0];
        Theta2[1] = theta2[1];
    }

    if (State[2]==1&&State[3]==1)
    {
        f1 = 47.63f * cosf(Theta3[2]) - 185.0f * sinf(Theta3[2]) + 200.0f;
        f2 = 47.63f * sinf(Theta3[2]) + 185.0f * cosf(Theta3[2]);

        a_coeff = -f2;
        b_coeff = -f1;
        c_const = z_target;

        float theta2[2]={0,0};
        if (Solve_Trig(a_coeff, b_coeff, c_const, theta2)==false)
        {
            State[2]=0;
            State[3]=0;
        }
        else if (theta2[0]==theta2[1])
        {
            State[2]=1;
            State[3]=0;
        }
        Theta2[2] = theta2[0];
        Theta2[3] = theta2[1];
    }
}

/**
 * @brief 求解第一关节角 θ₁
 * @param NULL
 * @return NULL
 * @note 为每个有效的 (θ₂, θ₃) 组合计算 θ₁
 */
void Inverse_Kinematics::Solve_Theta1(void)
{
    for (int i = 0; i < 4; ++i)
    {
        if (State[i] == 0)
        {
            continue;
        }

        float theta3, theta2;

        theta3 = Theta3[i];
        theta2 = Theta2[i];

        float f1 = 47.63f * cosf(theta3) - 185.0f * sinf(theta3) + 200.0f;
        float f2 = 47.63f * sinf(theta3) + 185.0f * cosf(theta3);

        float g1 = cosf(theta2) * f1 - sinf(theta2) * f2;

        if (fabsf(g1) < 1e-5f)
        {
            Theta1[i] = 0.0f;  // 奇异位置，取θ₁=0作为可行解
        }
        else
        {
            float ct1 = Wrist_Vector[0] / g1;
            float st1 = Wrist_Vector[1] / g1;

            float norm_sq = ct1 * ct1 + st1 * st1;
            const float TOLERANCE = 1e-3f; // 容差

            // 检查是否有限且接近单位圆
            if (!std::isfinite(ct1) || !std::isfinite(st1) || fabsf(norm_sq - 1.0f) > TOLERANCE)
            {
                State[i] = 0;
            }
            else
            {
                Theta1[i] = atan2f(st1, ct1);
            }
        }
    }
}

/**
 * @brief 计算两个旋转矩阵的乘积
 * @param A 第一个旋转矩阵
 * @param B 第二个旋转矩阵
 * @return 旋转矩阵乘积 C = A × B
 */
Inverse_Kinematics::Rotation_Matrix Inverse_Kinematics::RotationMatrix_Multiply(const Rotation_Matrix& A, const Rotation_Matrix& B)
{
    Rotation_Matrix C;
    for (uint8_t i = 0; i < 3; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            C.m[i][j] = 0.0f;
            for (uint8_t k = 0; k < 3; ++k)
            {
                C.m[i][j] += A.m[i][k] * B.m[k][j];
            }
        }
    }
    return C;
}

/**
 * @brief 计算旋转矩阵的转置
 * @param R 输入旋转矩阵
 * @return 转置后的旋转矩阵
 */
Inverse_Kinematics::Rotation_Matrix Inverse_Kinematics::RotationMatrix_Transpose(const Rotation_Matrix& R)
{
    Rotation_Matrix R1;
    for (uint8_t i=0;i<3;i++)
    {
        for (uint8_t j=0;j<3;j++)
        {
            R1.m[i][j]=R.m[j][i];
        }
    }
    return R1;
}

/**
 * @brief 根据DH参数计算单个连杆的旋转矩阵
 * @param m 连杆编号 (1-6)
 * @param theta 关节角度
 * @return 对应连杆的旋转矩阵
 */
Inverse_Kinematics::Rotation_Matrix Inverse_Kinematics::Compute_Link_R(uint8_t m, float theta)
{
    Rotation_Matrix R;
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(DH_Params[m-1].alpha);
    float sa = sinf(DH_Params[m-1].alpha);

    R.m[0][0] = ct;      R.m[0][1] = -st;     R.m[0][2] = 0;
    R.m[1][0] = st*ca;   R.m[1][1] = ct*ca;   R.m[1][2] = -sa;
    R.m[2][0] = st*sa;   R.m[2][1] = ct*sa;   R.m[2][2] = ca;

    return R;
}

/**
 * @brief 将ZYZ欧拉角转换为旋转矩阵
 * @param Coor_Pos 包含ZYZ欧拉角 (alpha, beta, gamma) 的位姿
 * @return 对应的3x3旋转矩阵
 */
Inverse_Kinematics::Rotation_Matrix Inverse_Kinematics::ZYZ_to_RotationMatrix(const Coordinates_Pose& Coor_Pos)
{
    float phi = Coor_Pos.alpha;   // Z
    float theta = Coor_Pos.beta; // Y
    float psi = Coor_Pos.gamma;   // Z'

    float c1 = cosf(phi), s1 = sinf(phi);
    float c2 = cosf(theta), s2 = sinf(theta);
    float c3 = cosf(psi), s3 = sinf(psi);

    Rotation_Matrix R;
    // 标准 Z-Y-Z 欧拉角旋转矩阵公式
    R.m[0][0] = c1*c2*c3 - s1*s3;  R.m[0][1] = -c1*c2*s3 - s1*c3; R.m[0][2] = c1*s2;
    R.m[1][0] = s1*c2*c3 + c1*s3;  R.m[1][1] = -s1*c2*s3 + c1*c3; R.m[1][2] = s1*s2;
    R.m[2][0] = -s2*c3;            R.m[2][1] = s2*s3;             R.m[2][2] = c2;

    return R;
}

/**
 * @brief 求解腕部关节角 θ₄, θ₅, θ₆
 * @param Coor_Pos 目标末端位姿
 * @return NULL
 * @note 为每个有效的前三关节解计算对应的腕部关节角
 */
void Inverse_Kinematics::Solve_Theta456(const Coordinates_Pose& Coor_Pos,const float CurrentTheta4)
{
    const float EPS = 1e-6f;
    const float SINGULAR_TOL = 0.05f;

    Rotation_Matrix R06 = ZYZ_to_RotationMatrix(Coor_Pos);

    for (int i = 0; i < 4; ++i)
    {
        if (State[i] == 0)
        {
            continue;
        }

        float theta1 = Theta1[i];
        float theta2 = Theta2[i];
        float theta3 = Theta3[i];

        Rotation_Matrix R01, R12, R23;
        Rotation_Matrix R02, R03;
        Rotation_Matrix R03_T;

        R01 = Compute_Link_R(1, theta1);
        R12 = Compute_Link_R(2, theta2);
        R23 = Compute_Link_R(3, theta3);

        R02 = RotationMatrix_Multiply(R01, R12);
        R03 = RotationMatrix_Multiply(R02, R23);
        R03_T = RotationMatrix_Transpose(R03);

        Rotation_Matrix R36 = RotationMatrix_Multiply(R03_T, R06);

        float r13 = R36.m[0][2];
        float r21 = R36.m[1][0];
        float r22 = R36.m[1][1];
        float r23 = R36.m[1][2];
        float r33 = R36.m[2][2];

        float cos_th5 = r23;

        if (cos_th5 > 1.0f) cos_th5 = 1.0f;
        if (cos_th5 < -1.0f) cos_th5 = -1.0f;

        float sin_sq_th5 = 1.0f - cos_th5 * cos_th5;
        if (sin_sq_th5 < 0.0f) sin_sq_th5 = 0.0f;
        float sin_th5 = sqrtf(sin_sq_th5);

        if (sin_th5 < SINGULAR_TOL)
        {
            if (cos_th5 > 0.5f)
            {
                Theta5[i] = 0.0f;
                Theta4[i] = CurrentTheta4;
                Theta6[i] = atan2f(-R36.m[0][1], R36.m[0][0]) - Theta4[i];
            }
            else
            {
                Theta5[i] = PI;
                Theta4[i] = CurrentTheta4;
                Theta6[i] = -atan2f(R36.m[0][1], R36.m[0][0]) - Theta4[i];
            }
        }
        else
        {
            // Theta4[i] = atan2f(r33, -r13);
            // Theta5[i] = atan2f(sin_th5, cos_th5);
            // Theta6[i] = atan2f(-r22, r21);

            float t4_a = atan2f(r33, -r13);
            float t5_a = atan2f(sin_th5, cos_th5);
            float t6_a = atan2f(-r22, r21);

            float t4_b = t4_a + PI;
            float t5_b = -t5_a;
            float t6_b = t6_a + PI;

            float diff_a = t4_a - CurrentTheta4;
            while (diff_a >  PI) diff_a -= 2.0f * PI;
            while (diff_a < -PI) diff_a += 2.0f * PI;

            float diff_b = t4_b - CurrentTheta4;
            while (diff_b >  PI) diff_b -= 2.0f * PI;
            while (diff_b < -PI) diff_b += 2.0f * PI;

            if (fabsf(diff_a) <= fabsf(diff_b))
            {
                Theta4[i] = t4_a;
                Theta5[i] = t5_a;
                Theta6[i] = t6_a;
            }
            else
            {
                Theta4[i] = t4_b;
                Theta5[i] = t5_b;
                Theta6[i] = t6_b;
            }

            // Theta4[i] += PI;   // θ₄' = θ₄ + π
            // Theta5[i] = -Theta5[i]; // θ₅' = -θ₅
            // Theta6[i] += PI;   // θ₆' = θ₆ + π
        }

        auto normalize_angle = [](float angle) -> float {
            angle = fmodf(angle, 2.0f * PI);
            if (angle > PI) angle -= 2.0f * PI;
            else if (angle < -PI) angle += 2.0f * PI;
            return angle;
        };

        Theta4[i] = normalize_angle(Theta4[i]);
        Theta5[i] = normalize_angle(Theta5[i]);
        Theta6[i] = normalize_angle(Theta6[i]);
    }
}

/**
 * @brief 从所有有效解中选择最优解
 * @param current_angles 当前关节角度数组
 * @param best_angles 输出最优关节角度数组
 * @return bool 成功找到最优解返回 true，否则返回 false
 * @note 根据加权平方距离选择与当前角度最接近的解
 */
bool Inverse_Kinematics::SelectBestSolution(const float current_angles[6],float best_angles[6])
{
    int best_index = -1;
    float min_weighted_distance = std::numeric_limits<float>::max(); // 初始化为一个很大的数

    for (int i = 0; i < 4; ++i)
    {
        if (State[i] == 0)
        {
            continue;
        }

        float weighted_distance = 0.0f;

        const float* solution_angles[6] = {&Theta1[i], &Theta2[i], &Theta3[i],
                                           &Theta4[i], &Theta5[i], &Theta6[i]};

        for (int j = 0; j < 6; ++j)
        {
            float angle_diff = *(solution_angles[j]) - current_angles[j];

            // 角度差归一化到 [-π, π] 区间，避免因 ±2π 的等价性导致距离计算错误
            while (angle_diff > PI) angle_diff -= 2 * PI;
            while (angle_diff < -PI) angle_diff += 2 * PI;

            weighted_distance += DEFAULT_WEIGHTS[j] * (angle_diff * angle_diff);
        }

        if (last_best_index != -1 && State[last_best_index] != 0 && i != last_best_index)
        {
            weighted_distance *= SWITCH_PENALTY; // 如果不是上一次的解，惩罚它的距离
        }

        if (weighted_distance < min_weighted_distance)
        {
            min_weighted_distance = weighted_distance;
            best_index = i;
        }

    }

    if (best_index != -1)
    {
        last_best_index = best_index;

        best_angles[0] = Theta1[best_index];
        best_angles[1] = Theta2[best_index];
        best_angles[2] = Theta3[best_index];
        best_angles[3] = Theta4[best_index];
        best_angles[4] = Theta5[best_index];
        best_angles[5] = Theta6[best_index];

        // float target_angles[6] = {
        //     Theta1[best_index], Theta2[best_index], Theta3[best_index],
        //     Theta4[best_index], Theta5[best_index], Theta6[best_index]
        // };

        // for (int j = 0; j < 6; ++j)
        // {
        //
        //     float diff = target_angles[j] - current_angles[j];
        //
        //     while (diff > PI) diff -= 2 * PI;
        //     while (diff < -PI) diff += 2 * PI;
        //
        //     best_angles[j] = current_angles[j] + diff;
        // }

        for (int j = 0; j < 6; ++j)
        {
            while (best_angles[j] >  PI) best_angles[j] -= 2.0f * PI;
            while (best_angles[j] < -PI) best_angles[j] += 2.0f * PI;
        }

        return true;
    }
    else
    {
        memset(best_angles, 0, sizeof(float) * 6);
        return false;
    }


}

/**
 * @brief 完整的逆运动学求解接口
 * @param target_pose 目标末端位姿
 * @param current_angles 当前关节角度
 * @param best_angles 输出最优关节角度
 * @return bool 求解成功返回 true，失败返回 false
 */
bool Inverse_Kinematics::Solve_FinalTheta(const Coordinates_Pose& target_pose,
    const float current_angles[6],float best_angles[6],const float CurrentTheta4)
{
    if (!current_angles || !best_angles)
    {
        return false;
    }

    Clear();

    Wrist_Position_Optimized(target_pose);

    Solve_Theta3();
    Solve_Theta2();
    Solve_Theta1();
    Solve_Theta456(target_pose,CurrentTheta4);

    if (SelectBestSolution(current_angles,best_angles))return true;
    return false;
}
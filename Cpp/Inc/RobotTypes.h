//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_ROBOTTYPES_H
#define MECHANICALARM_CODE_V2_ROBOTTYPES_H

#include <cstdint>

static constexpr float PI = 3.14159265358979323846f;

//DH参数
typedef struct
{
    float alpha;
    float a;
    float d;
    float theta;
}DH_Params_t;

//位姿参数
typedef struct
{
    float x;
    float y;
    float z;
    float alpha;
    float beta;
    float gamma;
}Coordinates_Pose;

//旋转矩阵
typedef struct
{
    float m[3][3];
}Rotation_Matrix;

//手柄参数
typedef struct
{
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;

    uint8_t lt;
    uint8_t rt;

    uint8_t btn_a;
    uint8_t btn_b;
    uint8_t btn_x;
    uint8_t btn_y;
    uint8_t btn_lb;
    uint8_t btn_rb;
    uint8_t btn_m1;
    uint8_t btn_m2;

    int8_t dpad_x;
    int8_t dpad_y;


}Pad_Params_t;

//手柄控制模式
enum Pad_Mode
{
    Standby, 
    Cartesian,
    Joint,
    Tool,
    MODE_COUNT
};

//手柄控制锁定信息
enum Pad_Lock
{
    None,
    Lock1,
    Lock2,
    Lock3,
    Lock4,
    Lock5,
    Lock6,
    LOCK_COUNT
};

//任务名称
enum TaskName
{
    NONE,
    PTP,
    PAD
};

static constexpr DH_Params_t DH_Params[6]=
{{0,0,161,0},
 {-PI/2,0,0,-PI/2},
 {0,200,0,PI/2},
 {-PI/2,47.63,185,0},
 {PI/2,0,0,-PI/2},
 {-PI/2,0,130,0}
};

//回零位姿
static constexpr Coordinates_Pose Home={177.63f,0.0f,176.0f,0.0f,1.57f,3.14f};



#endif //MECHANICALARM_CODE_V2_ROBOTTYPES_H
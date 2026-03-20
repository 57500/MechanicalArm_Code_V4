//
// Created by 57500 on 2026/3/2.
//

#ifndef MECHANICALARM_CODE_V2_ROBOTTYPES_H
#define MECHANICALARM_CODE_V2_ROBOTTYPES_H

static constexpr float PI = 3.14159265358979323846f;

typedef struct
{
    float alpha;
    float a;
    float d;
    float theta;
}DH_Params_t;

typedef struct
{
    float x;
    float y;
    float z;
    float alpha;
    float beta;
    float gamma;
}Coordinates_Pose;

static constexpr DH_Params_t DH_Params[6]=
{{0,0,161,0},
 {-PI/2,0,0,-PI/2},
 {0,200,0,PI/2},
 {-PI/2,47.63,185,0},
 {PI/2,0,0,-PI/2},
 {-PI/2,0,130,0}
};



#endif //MECHANICALARM_CODE_V2_ROBOTTYPES_H
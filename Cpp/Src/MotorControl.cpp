//
// Created by 57500 on 2026/2/28.
//
#include "MotorControl.h"

#include <sys/_stdint.h>
#include "can.h"
#include <cmath>

#include "usart.h"


MotorControl::MotorControl(uint8_t Addr)
{
    if (Addr < 1 || Addr > 6)
    {
        Addr=0;
    }
    this->Addr = Addr;

}

/**
 * @brief 电机使能控制
 * @param 地址
 * @return NULL
 */
void MotorControl::Motor_Enable(void)
{
    uint8_t Data[6]={0};
    Data[0]=Addr;
    Data[1]=0xF3;
    Data[2]=0xAB;
    Data[3]=0x01;
    Data[4]=0x00;
    Data[5]=0x6B;
    MyCAN1_Transmit(Data,6);
}

/**
 * @brief 速度模式控制
 * @param Addr   电机地址
 * @param Speed  目标速度，正负表示方向，绝对值范围0~100.0
 *               注意：实际转换为RPM会根据不同地址使用不同系数
 * @param Acc    加速度 0~255, 0为直接启动
 * @param Syn    同步标志
 * @return NULL
 */
void MotorControl::Motor_VelControl(float Speed, uint8_t Acc, uint8_t Syn)
{

    //获取减速比系数
    float ratio = RATIOS[Addr];

    //方向
    uint8_t Dir = (Speed >= 0.0f) ? 0x01 : 0x00;

    float absSpeed = fabsf(Speed);
    float rpmFloat = absSpeed * ratio;

    //限制最大转速
    const uint16_t MAX_RPM = 1000;
    if (rpmFloat > MAX_RPM) {
        rpmFloat = MAX_RPM;
    }

    //四舍五入转换为整数
    uint16_t Rpm = (uint16_t)(rpmFloat + 0.5f);

    const uint16_t RPM_DEADBAND = 2;
    if (Rpm <= RPM_DEADBAND)
    {
        Rpm=0;
    }


    uint8_t Data[8] = {0};
    Data[0] = Addr;
    Data[1] = 0xF6;
    Data[2] = Dir;
    Data[3] = (uint8_t)((Rpm >> 8) & 0xFF);  // 高字节
    Data[4] = (uint8_t)(Rpm & 0xFF);         // 低字节
    Data[5] = Acc;
    Data[6] = Syn;
    Data[7] = 0x6B;
    MyCAN1_Transmit(Data, 8);
}

/**
  * @brief    位置模式控制
  * @param    电机地址
  * @param    方向        ，0为CW，其余值为CCW
  * @param    速度(RPM)   ，范围0 - 1000RPM
  * @param    加速度      ，范围0 - 255，注意：0是直接启动
  * @param    脉冲数      ，范围0- (2^32 - 1)个,3200为一圈
  * @param    相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    多机同步标志 ，false为不启用，true为启用
  * @return   NULL
  */
void MotorControl::Motor_PosControl(uint16_t Vel,
    uint8_t Acc,float Rad,uint8_t RaF,uint8_t Syn)
{
    uint8_t Data[13]={0};

    if (Vel>1000)
    {
        Vel = 1000;
    }

    //获取减速比系数

    float ratio = RATIOS[Addr];

    //方向
    uint8_t Dir = (Rad >= 0.0f) ? 0x01 : 0x00;

    float absRad = fabsf(Rad);
    float ClkFloat = absRad * ratio*3200.0f/(2.0f*PI);

    uint32_t Clk = (uint32_t)(ClkFloat + 0.5f);

    Data[0]=Addr;
    Data[1]=0xFD;
    Data[2]=Dir;
    Data[3]=(uint8_t)(Vel>>8);
    Data[4]=(uint8_t)(Vel>>0);
    Data[5]=Acc;
    Data[6]=(uint8_t)(Clk >> 24);
    Data[7]=(uint8_t)(Clk >> 16);
    Data[8]=(uint8_t)(Clk >> 8);
    Data[9]=(uint8_t)(Clk >> 0);
    Data[10]=RaF;
    Data[11]=Syn;
    Data[12]=0x6B;
    MyCAN1_Transmit(Data,13);
}

/**
  * @brief    读取系统参数
  * @param    Addr  ：电机地址
  * @param    Param ：系统参数类型
  * 0x35--实时速度
  * 0x36--实时位置
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void MotorControl::Read_Motor_Param(uint8_t Param)
{
    uint8_t Data[3]={0};
    Data[0]=Addr;
    Data[1]=Param;
    Data[2]=0x6B;
    MyCAN1_Transmit(Data,3);
}

/**
 * @brief  同步启动模式
 * @param  NuLL
 * @return NULL
 */
void MotorControl::Motor_SynMode(void)
{
    uint8_t Data[4]={0};
    Data[0]=0x00;
    Data[1]=0xFF;
    Data[2]=0x66;
    Data[3]=0x6B;
    MyCAN1_Transmit(Data,4);
}


/**
 * @brief 修改电机地址
 * @param Addr        :原地址
 * @param Change_Addr :目标地址
 * @return NULL
 */
void MotorControl::Change_MotorAddr(uint8_t Change_Addr)
{
    uint8_t Data[6]={0};
    Data[0]=Addr;
    Data[1]=0xAE;
    Data[2]=0x4B;
    Data[3]=0x01;
    Data[4]=Change_Addr;
    Data[5]=0x6B;
    MyCAN1_Transmit(Data,6);
    Addr=Change_Addr;
}

void MotorControl::Read_Motor_Pos(uint8_t addr)
{
    uint8_t Data[3]={0};
    Data[0]=addr;
    Data[1]=0x36;
    Data[2]=0x6B;
    MyCAN1_Transmit(Data,3);
}
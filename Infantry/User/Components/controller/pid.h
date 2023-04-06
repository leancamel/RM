/**
  *******************************RM Warrior 2023********************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-1-1        pxx             1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************RM Warrior 2023********************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA        //增量式PID
};

typedef struct
{
    uint8_t mode;
    //PID 参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;      //设定值
    fp32 fdb;      //实际值

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
#endif

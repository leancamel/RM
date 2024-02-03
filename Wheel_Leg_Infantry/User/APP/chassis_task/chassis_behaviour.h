/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  v2.0.0     Nov-05-2023     pxx             刚起步
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "main.h"

#include "chassis_task.h"

// #define LEG_POS_TEST_START // 腿部姿态控制测，使用PID控制腿部角度 不开启请注释
#define RC_LENGTH_CHANNEL CHASSIS_X_CHANNEL
#define RC_ANGLE_CHANNEL CHASSIS_WZ_CHANNEL
#define RC_LENGTH_SEN 0.0000378f
#define RC_ANGLE_SEN 0.0007923f

typedef enum
{
	CHASSIS_ZERO_FORCE,	   // 底盘无力
	CHASSIS_STAND_UP,	     // 机器人起立，中间过渡状态，由程序自动判断切换
	CHASSIS_NO_MOVE,	     // 底盘保持不动，如果机器人还未站起，则先进入机器人起立状态机，等待站起后自动切换到此状态机
	CHASSIS_NO_FOLLOW_YAW, // 底盘不跟随云台，旋转直接由遥控器设定
} chassis_behaviour_e;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *l_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif

#ifndef __ROS_REVEIVE_H
#define __ROS_RECEIVE_H

#include "main.h"
/**
 * @brief	接收结构体，存放底盘速度，以及云台到目标向量坐标
 * 坐标系为以枪管最前端中心点为坐标原点，沿枪管向前为 y 轴正方向，垂直枪管向上为 z 轴正方向的右手坐标系
 */
typedef struct
{
	fp32 gimbal_x;
	fp32 gimbal_y;
	fp32 gimbal_z;
	fp32 vx_set;
	fp32 vy_set;
	fp32 wz_set;
}ROS_Msg_Struct;
/**
 * @brief	发送结构体，发送里程计信息，以及是否正确接收
 */
typedef struct
{
	fp32 vx;				/*!< 电机反馈 vx 速度，单位 m/s */
	fp32 vy;				/*!< 电机反馈 vy 速度，单位 m/s */
	fp32 wz;				/*!< 电机反馈 wz 速度，单位 rad/s */
	unsigned char response;/*!< 判断是否正确接收，正确接收为 0x00 ，错误为 0xFF */
}ROS_Response_Struct;

void ROS_Msg_Init(void);
void Get_Chassis_Msg(fp32 *vx_set,fp32 *vy_set,fp32 *wz_set);
void Get_Gimbal_Msg(fp32 *gimbal_x,fp32 *gimbal_y,fp32 *gimbal_z);
void Pack_Response(fp32 vx,fp32 vy,fp32 wz);

#endif

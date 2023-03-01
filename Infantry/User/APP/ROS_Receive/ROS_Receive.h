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
}ROS_Msg_Struct;

void ROS_Msg_Init(void);
void Get_Gimbal_Msg(fp32 *gimbal_x,fp32 *gimbal_y,fp32 *gimbal_z);

#endif

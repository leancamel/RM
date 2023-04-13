#ifndef __ROS_REVEIVE_H
#define __ROS_RECEIVE_H

#include "main.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"

#define ROS_START_BYTE 0x5A
#define ROS_RX_BUF_NUM 48u
#define ROS_FRAME_LENGTH 24u

typedef union
{
	fp32 float_data;
	uint8_t byte_data[4];
}Float_Byte;

typedef struct
{
	Float_Byte vx_set;
	Float_Byte vy_set;
	Float_Byte wz_set;
	Float_Byte yaw_add;
	Float_Byte pitch_add;
	uint8_t mode;
	Float_Byte vx;
	Float_Byte vy;
	Float_Byte wz;
}ROS_Msg_t;

void ROS_Init(void);
void Pack_Response(fp32 vx,fp32 vy,fp32 wz);
void Get_Chassis_Msg(fp32 *vx_set,fp32 *vy_set,fp32 *angle_set);
void Get_Gimbal_Msg(fp32 *yaw_add,fp32 *pitch_add);
void Get_Chassis_Mode(chassis_behaviour_e *chassis_behaviour_mode);
void Get_Gimbal_Mode(gimbal_behaviour_e *gimbal_behaviour);
void Get_Shoot_Msg(bool_t *last_shoot_switch);
void ROS_Send_Msg(void);

#endif

#ifndef ROS_RECEIVE_H
#define ROS_RECEIVE_H

#include "main.h"

#define ROS_START_BYTE 0x42
#define ROS_RX_BUF_NUM 32u
#define ROS_FRAME_LENGTH 16u

//陀螺仪数据发送周期 ms
#define IMU_SEND_TIME 20


typedef union
{
	float float_data;
	uint8_t byte_data[4];
}Float_Byte;

typedef struct
{
	Float_Byte yaw_add;
	Float_Byte pitch_add;
	Float_Byte depth;
}ROS_Msg_t;

void imuSendTask(void *pvParameters);

void ROS_Init(void);
const ROS_Msg_t *get_ROS_Msg_point(void);
void Get_Gimbal_Angle(fp32 *yaw_add,fp32 *pitch_add);

#endif

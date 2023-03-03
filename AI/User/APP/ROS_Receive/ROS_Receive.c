#include "stm32f4xx.h"
#include "ROS_Receive.h"
#include "uart1.h"
#include <stdio.h>
#include <stdarg.h>

#define CRC_Len 2		// CRC 校验位长度
#define Packet_Len 14	//接收数据包长，12位数据位 + 2位 CRC 校验位
#define Start_Byte 0xFF	//接收起始帧
#define Response_Start_Byte_1 0x55	//发送起始帧1
#define Response_Start_Byte_2 0xAA	//发送起始帧2
ROS_Msg_Struct ROS_Msg;
ROS_Response_Struct ROS_Response;

uint8_t Serial_RxPacket[Packet_Len];

uint8_t CRC_Data[Packet_Len];
/**
 * @brief	初始化发送、接收结构体数据初始化
 * @return	void
 */
void ROS_Msg_Init(void)
{
	ROS_Msg.gimbal_x = 0;
	ROS_Msg.gimbal_y = 0;
	ROS_Msg.gimbal_z = 0;
	ROS_Msg.vx_set = 0;
	ROS_Msg.vy_set = 0;
	ROS_Msg.wz_set = 0;

	ROS_Response.response = 0x00;
	return;
}
/**
 * @brief	将接受到的 int 数据除以 1000 后转换需要的 float 数据
 * @param	p_Byte	传入的 Byte 数组指针
 * @return	转换后的 float 数据
 */
float Byte_To_Float(unsigned char *p_Byte,unsigned char offest)
{
	short temp_int = (*(p_Byte + offest) << 8) | *(p_Byte + offest + 1);
	return (float)temp_int / 1000;
}
/**
 * @brief	将接受到的数据封装进结构体
 * @return	void
 */
void Pack_Msg(void)
{
	ROS_Msg.gimbal_x = Byte_To_Float(CRC_Data,0);
	ROS_Msg.gimbal_y = Byte_To_Float(CRC_Data,2);
	ROS_Msg.gimbal_z = Byte_To_Float(CRC_Data,4);
	ROS_Msg.vx_set = Byte_To_Float(CRC_Data,6);
	ROS_Msg.vy_set = Byte_To_Float(CRC_Data,8);
	ROS_Msg.wz_set = Byte_To_Float(CRC_Data,10);
	return;
}
/**
 * @brief	获取接受到的底盘速度信息
 * @param	vx_set	m/s
 * @param	vy_set	m/s
 * @param	wz_set	rad/s
 * @return	void
 */
void Get_Chassis_Msg(fp32 *vx_set,fp32 *vy_set,fp32 *wz_set)
{
	*vx_set = ROS_Msg.vx_set;
	*vy_set = ROS_Msg.vy_set;
	*wz_set = ROS_Msg.wz_set;
}
/**
 * @brief	获取接受到的云台控制信息
 * @param	vx_set	m/s
 * @param	vy_set	m/s
 * @param	wz_set	rad/s
 * @return	void
 */
void Get_Gimbal_Msg(fp32 *gimbal_x,fp32 *gimbal_y,fp32 *gimbal_z)
{
	*gimbal_x = ROS_Msg.gimbal_x;
	*gimbal_y = ROS_Msg.gimbal_y;
	*gimbal_z = ROS_Msg.gimbal_z;
}
/**
 * @brief	将待发送数据封装进结构体
 * @param	vx	m/s
 * @param	vy	m/s
 * @param	wz	rad/s
 * @return	void
 */
void Pack_Response(fp32 vx,fp32 vy,fp32 wz)
{
	ROS_Response.vx = vx;
	ROS_Response.vy = vy;
	ROS_Response.wz = wz;
	return;
}
/**
 * @brief	将 float 数据乘 1000 转换为 short 数据发出
 * @param 	msg	待发送的 float 数据
 * @return 	void
 */
void Send_Float(fp32 msg)
{
	short temp_short = msg * 1000;
	Serial_SendByte((temp_short & 0xFF00) >> 8);
	Serial_SendByte(temp_short & 0x00FF);
	return;
}
/**
 * @brief	将 ROS_Response 结构体内的数据发出
 * @return	void
 */
void Msg_Response(void)
{
	Serial_SendByte(Response_Start_Byte_1);
	Serial_SendByte(Response_Start_Byte_2);
	Send_Float(ROS_Response.vx);
	Send_Float(ROS_Response.vy);
	Send_Float(ROS_Response.wz);
	Serial_SendByte(ROS_Response.response);
	return;
}
/**
 * @brief	计算 CRC 校验码
 * @param	_pBuf	待计算的数组指针
 * @param	_usLen	待计算的数据长度
 * @return 	void
 */
void getModbusCRC16(unsigned char *_pBuf, unsigned short int _usLen)
{
    unsigned short int CRCValue = 0xFFFF;
    unsigned char i,j;

    for(i=0;i<_usLen;++i)
    {
        CRCValue  ^= *(_pBuf+i);
        for(j=0;j<8;++j)
        {
            if((CRCValue & 0x01) == 0x01)
            {
                 CRCValue = (CRCValue >> 1)^0xA001;
            }
            else 
            {
                CRCValue >>= 1;
            }           
        }
    } 
    *(_pBuf + _usLen) = (CRCValue & 0xFF00) >> 8; 		// CRC 校验码高位
    *(_pBuf + _usLen + 1) = CRCValue & 0x00FF;			// CRC 校验码低位
    return;            
} 
//串口中断函数
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		if (RxState == 0)
		{
			if (RxData == Start_Byte)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			Serial_RxPacket[pRxPacket] = RxData;
			CRC_Data[pRxPacket] = RxData;
			pRxPacket ++;
			if (pRxPacket >= Packet_Len)
			{
				getModbusCRC16(CRC_Data,Packet_Len - CRC_Len);
				if((CRC_Data[12] == Serial_RxPacket[12]) && (CRC_Data[13] == Serial_RxPacket[13]))
				{
					Pack_Msg();
					ROS_Response.response = 0x00;
				}
				else
				{
					ROS_Response.response = 0xFF;
				}
				RxState = 0;
				Msg_Response(); 
			}
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

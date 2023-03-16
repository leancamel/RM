#include "stm32f4xx.h"
#include "ROS_Receive.h"
#include "uart1.h"
#include <stdio.h>
#include <stdarg.h>

#define CRC_Len 2		// CRC 校验位长度
#define Packet_Len 8	//接收数据包长，6位数据位 + 2位 CRC 校验位
#define Start_Byte 0xFF	//接收起始帧
ROS_Msg_Struct ROS_Msg;

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
	return;
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
				if((CRC_Data[6] == Serial_RxPacket[6]) && (CRC_Data[7] == Serial_RxPacket[7]))
				{
					Pack_Msg();
					Serial_SendByte(0x00);//正确接收发送 0x00
				}
				else
				{
					Serial_SendByte(0xFF);//错误接收发送 0xFF
				}
				RxState = 0;
			}
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

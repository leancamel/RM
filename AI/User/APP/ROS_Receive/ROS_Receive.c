#include "stm32f4xx.h"
#include "ROS_Receive.h"
#include "uart1.h"
#include <stdio.h>
#include <stdarg.h>
#include "detect_task.h"
//ROS出错数据上限
#define ROS_Receive_ERROR_VALUE 500

static uint8_t ROS_rx_buf[2][ROS_RX_BUF_NUM];
static ROS_Msg_t ROS_Msg;

//将串口接收到的数据转化为ROS实际的数据
void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg);
//发送速度信息给ROS
void ROS_Send_Msg(void);
//初始化DMA，串口1
void ROS_Init(void)
{
    ROS_Receive_Init(ROS_rx_buf[0], ROS_rx_buf[1], ROS_RX_BUF_NUM);
	ROS_Msg.vx_set.float_data = 0.0f;
	ROS_Msg.vy_set.float_data = 0.0f;
	ROS_Msg.wz_set.float_data = 0.0f;
	ROS_Msg.yaw_add.float_data = 0.0f;
	ROS_Msg.pitch_add.float_data = 0.0f;
	ROS_Msg.mode = 0x00;
	ROS_Msg.vx.float_data = 0.0f;
	ROS_Msg.vy.float_data = 0.0f;
	ROS_Msg.wz.float_data = 0.0f;
}
/**
 * @brief	将待发送数据封装进结构体,若传入的数据大于等于10,则不进行更新
 * @param	vx	m/s
 * @param	vy	m/s
 * @param	wz	rad/s
 * @return	void
 */
void Pack_Response(fp32 vx,fp32 vy,fp32 wz)
{
	if(vx < 10)
		ROS_Msg.vx.float_data = vx;
	if(vy < 10)
		ROS_Msg.vy.float_data = vy;
	if(wz < 10)
		ROS_Msg.wz.float_data = wz;
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
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = ROS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5);
            DMA_SetCurrDataCounter(DMA2_Stream5, ROS_RX_BUF_NUM);
            DMA2_Stream5->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == ROS_FRAME_LENGTH)
            {
                //处理ROS数据
				UART_to_ROS_Msg(ROS_rx_buf[0], &ROS_Msg);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = ROS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5); 
            DMA_SetCurrDataCounter(DMA2_Stream5, ROS_RX_BUF_NUM);
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == ROS_FRAME_LENGTH)
            {
                //处理ROS数据
				UART_to_ROS_Msg(ROS_rx_buf[1], &ROS_Msg);
            }
        }
    }
}

void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg)
{
	if(uart_buf == NULL || ros_msg == NULL || uart_buf[0] != ROS_START_BYTE)
	{
		return;
	}
    uint8_t CRC1 = *(uart_buf + 22);
    uint8_t CRC2 = *(uart_buf + 23);
	getModbusCRC16(uart_buf + 1,21);

	if(CRC1 == *(uart_buf + 22) && CRC2 == *(uart_buf + 23))
    {
        ros_msg->vx_set.byte_data[0] = *(uart_buf + 1);
        ros_msg->vx_set.byte_data[1] = *(uart_buf + 2);
        ros_msg->vx_set.byte_data[2] = *(uart_buf + 3);
        ros_msg->vx_set.byte_data[3] = *(uart_buf + 4);

		ros_msg->vy_set.byte_data[0] = *(uart_buf + 5);
        ros_msg->vy_set.byte_data[1] = *(uart_buf + 6);
        ros_msg->vy_set.byte_data[2] = *(uart_buf + 7);
        ros_msg->vy_set.byte_data[3] = *(uart_buf + 8);

		ros_msg->wz_set.byte_data[0] = *(uart_buf + 9);
        ros_msg->wz_set.byte_data[1] = *(uart_buf + 10);
        ros_msg->wz_set.byte_data[2] = *(uart_buf + 11);
        ros_msg->wz_set.byte_data[3] = *(uart_buf + 12);

		ros_msg->yaw_add.byte_data[0] = *(uart_buf + 13);
		ros_msg->yaw_add.byte_data[1] = *(uart_buf + 14);
		ros_msg->yaw_add.byte_data[2] = *(uart_buf + 15);
		ros_msg->yaw_add.byte_data[3] = *(uart_buf + 16);

		ros_msg->pitch_add.byte_data[0] = *(uart_buf + 17);
		ros_msg->pitch_add.byte_data[1] = *(uart_buf + 18);
		ros_msg->pitch_add.byte_data[2] = *(uart_buf + 19);
		ros_msg->pitch_add.byte_data[3] = *(uart_buf + 20);

		ros_msg->mode = *(uart_buf + 21);
    }
}

void Get_Chassis_Msg(fp32 *vx_set,fp32 *vy_set,fp32 *angle_set)
{
	*vx_set = ROS_Msg.vx_set.float_data;
	*vy_set = ROS_Msg.vy_set.float_data;
	*angle_set = ROS_Msg.wz_set.float_data;
}

void Get_Gimbal_Msg(fp32 *yaw_add,fp32 *pitch_add)
{
	*yaw_add = ROS_Msg.yaw_add.float_data;
	*pitch_add = ROS_Msg.pitch_add.float_data;
}

void Get_Chassis_Mode(chassis_behaviour_e *chassis_behaviour_mode)
{
	if(chassis_behaviour_mode != NULL)
	{
		switch(ROS_Msg.mode & 0x03)
		{
			case 0x00:
				*chassis_behaviour_mode = CHASSIS_NO_MOVE;
				break;
			case 0x01:
				*chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
				break;
			case 0x02:
				*chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
				break;
			default:
				*chassis_behaviour_mode = CHASSIS_NO_MOVE;
				break;
		}
	}
}

void Get_Gimbal_Mode(gimbal_behaviour_e *gimbal_behaviour)
{
	if(gimbal_behaviour != NULL)
	{
		switch(ROS_Msg.mode & 0x03)
		{
			case 0x00:
				*gimbal_behaviour = GIMBAL_ZERO_FORCE;
				break;
			case 0x01:
				*gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
				break;
			case 0x02:
				*gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
				break;
			default:
				*gimbal_behaviour = GIMBAL_ZERO_FORCE;
				break;
		}
	}
}

void ROS_Send_Msg(void)
{
	Serial_SendByte(ROS_START_BYTE);
	uint8_t CRC_Buff[14];

	CRC_Buff[0] = ROS_Msg.vx.byte_data[0];
	CRC_Buff[1] = ROS_Msg.vx.byte_data[1];
	CRC_Buff[2] = ROS_Msg.vx.byte_data[2];
	CRC_Buff[3] = ROS_Msg.vx.byte_data[3];

	CRC_Buff[4] = ROS_Msg.vy.byte_data[0];
	CRC_Buff[5] = ROS_Msg.vy.byte_data[1];
	CRC_Buff[6] = ROS_Msg.vy.byte_data[2];
	CRC_Buff[7] = ROS_Msg.vy.byte_data[3];

	CRC_Buff[8] = ROS_Msg.wz.byte_data[0];
	CRC_Buff[9] = ROS_Msg.wz.byte_data[1];
	CRC_Buff[10] = ROS_Msg.wz.byte_data[2];
	CRC_Buff[11] = ROS_Msg.wz.byte_data[3];

	getModbusCRC16(CRC_Buff,12);

	for(int i=0;i<14;i++)
	{
		Serial_SendByte(CRC_Buff[i]);
	}
}

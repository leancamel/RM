#include "stm32f4xx.h"
#include "ROS_Receive.h"
#include "uart1.h"

//ROS出错数据上限
#define ROS_Receive_ERROR_VALUE 500

static uint8_t ROS_rx_buf[2][ROS_RX_BUF_NUM];
static ROS_Msg_t ROS_Msg;

//将串口接收到的数据转化为ROS实际的数据
void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg);


//初始化DMA，串口1
void ROS_Init(void)
{
    ROS_Receive_Init(ROS_rx_buf[0], ROS_rx_buf[1], ROS_RX_BUF_NUM);
}

//串口中断
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

float Byte_To_Float(uint8_t *p_Byte, uint8_t offset)
{
    short temp_int = (*(p_Byte + offset) << 8) | *(p_Byte + offset + 1);
    return (float)temp_int / 1000;
}

void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg)
{
	if(uart_buf == NULL || ros_msg == NULL || uart_buf[0] != ROS_START_BYTE)
	{
		return;
	}

    uint8_t CRC1 = *(uart_buf + 7);
    uint8_t CRC2 = *(uart_buf + 8);
	getModbusCRC16(uart_buf + 1,6);

	if(CRC1 == *(uart_buf + 7) && CRC2 == *(uart_buf + 8))
    {
        ROS_Msg.gimbal_x = Byte_To_Float(uart_buf + 1, 0);
        ROS_Msg.gimbal_y = Byte_To_Float(uart_buf + 1, 2);
        ROS_Msg.gimbal_z = Byte_To_Float(uart_buf + 1, 4);
    }
}



void Get_Gimbal_Angle(fp32 *yaw,fp32 *pitch)
{
	*yaw = ROS_Msg.gimbal_x;
	*pitch = ROS_Msg.gimbal_z;
    // ROS_Msg.gimbal_x = 0.0f;
    // ROS_Msg.gimbal_y = 0.0f;
    // ROS_Msg.gimbal_z = 0.0f;
}

//返回ROS数据，通过指针传递方式传递信息
const ROS_Msg_t *get_ROS_Msg_point(void)
{
    return &ROS_Msg;
}


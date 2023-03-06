#include "uart1.h"
#include "stm32f4xx.h"

#include "stdio.h"
#include "stdarg.h"

/*------------------printf函数重定向-------------------*/
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (unsigned char) ch);
    while (!(USART1->SR & USART_FLAG_TXE));
    return (ch);
}
/*-----------------------------------------------------*/
//UART1_Tx PA9    UART1_Rx PB7
void UART1_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void serial_sent_Android_receive(char *string, ...)
{
    Serial_SendByte(0XA5);      //发送数据包头

    int a;                      //用来接收整形变量 
    fp32 b;                     //用来接收浮点型数据

    char *c = string;           //避免直接使用string，导致string指向的地址发生变化 
    va_list ap;                 //定义一个va_list类型变量ap
    va_start(ap, string);       //初始化va，使va指向string的下一个地址 

    uint8_t sum = 0x00;         //定义和校验数据位
    uint8_t temp_buff = 0;      //处理32位转换为8位

    void* t = &b;               //空指针转换
    uint8_t* m = (uint8_t*)t;   //类型转换

    while(*c != '\0')
    {
        if(*c == '%')                  //格式控制符 
        {
            switch(*++c)
            {
                case 'd':
                    a = va_arg(ap, int);  //检索变量
                    for (int i = 0; i < 4; i++)
                    {
                        temp_buff = (uint32_t)a & (0xFF << 8*i);
                        sum += temp_buff;
                        Serial_SendByte(temp_buff);
                    }
                    break;
                case 'f':
                    b = va_arg(ap, float);
                    for (int i = 0; i < 4; i++)
                    {
                        temp_buff = m[i];
                        sum += temp_buff;
                        Serial_SendByte(temp_buff);
                    }
                    break;
            }
        }
        c++;  //指向下一个地址 
    }
    
    Serial_SendByte(sum);       //发送和校验
    Serial_SendByte(0x5A);      //发送数据包尾
    va_end(ap);                 //关闭ap
}


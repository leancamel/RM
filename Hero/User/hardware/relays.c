#include "stm32f4xx.h"
#include "relays.h"

void Relays_On(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void Relays_Off(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
}

void Relays_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	Relays_On();
	return;
}

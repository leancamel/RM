#include "stm32f4xx.h"
#include "main.h"

#include "delay.h"
#include "led.h"
#include "adc.h"
#include "buzzer.h"
#include "key.h"
#include "rc.h"
#include "uart1.h"
#include "can.h"
#include "trigger.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "start_task.h"
#include "calibrate_task.h"

#include "remote_control.h"
#include "CAN_receive.h"

#include "ROS_Receive.h"

void BSP_Init(void);

int main(void)
{
    BSP_Init();

	startTast();
	vTaskStartScheduler();

	while(1)
	{
        static double compare = 1000;
        if(compare < 1400)
        {
            compare += 0.1;
        }
        led_green_toggle();
        fric1_on((uint16_t)compare);
        fric2_on((uint16_t)compare);
        delay_ms(1);
	}
}

void BSP_Init(void)
{
	//中断组4
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	//初始化滴答时钟
    delay_init(configTICK_RATE_HZ);

    //LED灯初始化
	led_Init();

	//蜂鸣器初始化
	buzzer_init(500, 84);

	//自定义按键初始化
	key_Init();

	//遥控器初始化
	remote_control_init();

	//stm32 板载温度传感器初始化
    temp_ADC_init();

	//摩擦轮电机pwm初始化
	trigger_PWM_Init();
	
	//电池电源电压采集初始化
	voltage_ADC_init();

	//串口1初始化
	UART1_Init();

	//CAN通信初始化
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

	//上电校准，flash读取函数，把校准值放回对应参数
    // cali_param_init();

	//将接收ROS发送的结构体清零
	ROS_Msg_Init();
}


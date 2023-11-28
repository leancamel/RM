/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H
#include "main.h"
/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__)   ||  defined(__CC_ARM) ||  defined(__GNUC__)
	#include <stdint.h>
	extern uint32_t SystemCoreClock;
#endif



#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION 0 //使用非systick中断作为调度时钟

#define configUSE_PREEMPTION            1   //1使用抢占式内核，0使用协程
#define configUSE_TIME_SLICING          1   //1使能时间片调度(默认式使能的)
  
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1    //1启用特殊方法来选择下一个要运行的任务
                                                     //一般是硬件计算前导零指令，如果所使用的
                                                     //MCU没有这些硬件指令的话此宏应该设置为0！

#define configUSE_TICKLESS_IDLE	        0    //1启用低功耗tickless模式
#define configUSE_QUEUE_SETS	        1    //为1时启用队列
#define configUSE_IDLE_HOOK				0    //1，使用空闲钩子；0，不使用
#define configUSE_TICK_HOOK				0    //1，使用时间片钩子；0，不使用
#define configCPU_CLOCK_HZ				( SystemCoreClock ) //CPU频率
#define  configTICK_RATE_HZ				( ( TickType_t ) 1000 ) //时钟节拍频率，这里设置为1000，周期就是1ms
#define configMAX_PRIORITIES			( 32 )  //可使用的最大优先级
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 128 )  //空闲任务使用的堆栈大小
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 24 * 1024 ) )   //系统所有总的堆大小
#define configMAX_TASK_NAME_LEN			( 16 )  //任务名字字符串长度

#define configUSE_16_BIT_TICKS			0   //系统节拍计数器变量数据类型，
                                            //1表示为16位无符号整形，0表示为32位无符号整形



#define configIDLE_SHOULD_YIELD			1   //为1时空闲任务放弃CPU使用权给其他同优先级的用户任务
#define configUSE_TASK_NOTIFICATIONS    1   //为1时开启任务通知功能，默认开启
#define configUSE_MUTEXES				1   //为1时使用互斥信号量
#define configQUEUE_REGISTRY_SIZE		8   //不为0时表示启用队列记录，具体的值是可以
                                            //记录的队列和信号量最大数目。
#define configCHECK_FOR_STACK_OVERFLOW	0   //大于0时启用堆栈溢出检测功能，如果使用此功能
                                            //用户必须提供一个栈溢出钩子函数，如果使用的话
#define configUSE_RECURSIVE_MUTEXES		1   //为1时使用递归互斥信号量
#define configUSE_MALLOC_FAILED_HOOK	0   //1使用内存申请失败钩子函数
#define configUSE_APPLICATION_TASK_TAG	0
#define configUSE_COUNTING_SEMAPHORES	1   //为1时使用计数信号量
#define configSUPPORT_DYNAMIC_ALLOCATION 1  //支持动态内存申请


#define configUSE_TRACE_FACILITY		1   //为1启用可视化跟踪调试

#define configGENERATE_RUN_TIME_STATS	1   //为1时启用运行时间统计功能
extern volatile uint64_t FreeRTOSRunTimeTicks;
#define portGET_RUN_TIME_COUNTER_VALUE() (FreeRTOSRunTimeTicks)
extern void ConfigureTimeForRunTimeStats(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() ConfigureTimeForRunTimeStats()


#define configUSE_STATS_FORMATTING_FUNCTIONS	1       //与宏configUSE_TRACE_FACILITY同时为1时会编译下面3个函数
                                                        //prvWriteNameToBuffer(),vTaskList(),
                                                        //vTaskGetRunTimeStats()

#define INCLUDE_uxTaskGetStackHighWaterMark 1   //为1时可以通过下面这个函数来查询任务栈的剩余空间
                                                //UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);
/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		    0       //为1时启用协程，启用协程以后必须添加文件croutine.c
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )   //协程的有效优先级数目

/* Software timer definitions. */
#define configUSE_TIMERS				1       //为1时启用软件定时器
#define configTIMER_TASK_PRIORITY		( 2 )   //软件定时器优先级
#define configTIMER_QUEUE_LENGTH		10      //软件定时器队列长度
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE * 2 )    //软件定时器任务堆栈大小

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

#define INCLUDE_xTaskGetHandle          1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0xf     //中断最低优先级

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5       //系统可管理的最高中断优先级

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
	
/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }	
	
/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
//#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */


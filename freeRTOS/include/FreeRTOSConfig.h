/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined (__ICCARM__)||defined(__CC_ARM)||defined(__GNUC__)
	#include <stdint.h>
	extern uint32_t SystemCoreClock;
#endif

/// @brief 用于配置程序运行时间监控时钟
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()	ConfigureTimeForRunTimeStats()
/// @brief 用于任务运行时间监控
#define	portGET_RUN_TIME_COUNTER_VALUE() getFreeRTOSRunTimeTicks()

/** @brief 为1时使用抢占式调度器，为0时使用协程
		当使用协程时会在以下地方进行切换
		1.调用taskYIELD()时
		2.调用使任务进入阻塞态的API函数
		3.应用程序明确定义了在中断中执行上下文切换
	*/
#define configUSE_PREEMPTION			1
/**	@brief 为1时使能空闲Hook函数
	@code 
		void vApplicationTickHook(void)
	@endcode */
#define configUSE_IDLE_HOOK				0
/**	
	@brief 为1时使能心跳Hook函数
	@code 
		void vApplicationTickHook(void)
	@endcode */
#define configUSE_TICK_HOOK				0
///	@brief CPU时钟频率Hz
#define configCPU_CLOCK_HZ				( SystemCoreClock )
///	@brief 系统时钟频率Hz
#define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
///	@brief 任务的优先级数量
#define configMAX_PRIORITIES			( 5 )
///	@brief 任务的最小堆栈大小，单位字节
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 130 )
///	@brief 堆大小
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 75 * 1024 ) )
///	@brief 最大任务名长度
#define configMAX_TASK_NAME_LEN			( 20 )
/// @brief 启用可视化跟踪模式
#define configUSE_TRACE_FACILITY		1
///	@brief 为1时TickType_t为16位，为0时TickType_t为32位
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
///	@brief 为1时使能互斥信号量，相关函数会被编译
#define configUSE_MUTEXES				1
///	@brief 可注册队列信号量数目
#define configQUEUE_REGISTRY_SIZE		8
/**	
	@brief 为1时使能堆栈溢出Hook函数
	@code 
		void vApplicationStackOverflowHook(TaskHandle_t xTask,char* pcTaskName)
	@endcode */
#define configCHECK_FOR_STACK_OVERFLOW	1
///	@brief 为1时使能递归互斥信号量，相关函数会被编译
#define configUSE_RECURSIVE_MUTEXES		1
/**	
	@brief 为1时使能内存分配失败Hook函数
	@code 
		void vApplicationTickHook(void)
	@endcode */
#define configUSE_MALLOC_FAILED_HOOK	0
///	@brief 为1时相关函数会被编译
#define configUSE_APPLICATION_TASK_TAG	0
///	@brief 为1时使能计数信号量
#define configUSE_COUNTING_SEMAPHORES	1
/** @brief 为1时开启时间统计功能，可以统计任务运行时间信息
		https://blog.csdn.net/zhangxuechao_/article/details/79082217*/
#define configGENERATE_RUN_TIME_STATS	1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Software timer definitions.计时软件定义。 */
#define configUSE_TIMERS				1
#define configTIMER_TASK_PRIORITY		( 2 )
#define configTIMER_QUEUE_LENGTH		10
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE * 2 )

/**	@brief将以下定义设置为1以包含API函数，或者设置为0
	排除API函数。 */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0xf

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

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


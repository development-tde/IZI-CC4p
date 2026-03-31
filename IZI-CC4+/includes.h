/*
 * includes.h
 *
 * Created: 12-8-2021 21:30:05
 *  Author: Milo
 */ 


#ifndef INCLUDES_H_
#define INCLUDES_H_

//#define DCBM1_TRACE_ENABLE
#define BOD_ENABLED

#define IZIOUTPUT_SRC
#define IZIOUTPUT_SRC_IZI	0
#define IZIOUTPUT_SRC_DMX	1

#define DMA_CHANNELS		2

#define USART1_DMA_CHANNEL	0
#define USART1_DMA

#define DCBM1_UART_INIT				Usart1_Init
#define DCBM1_UART_PUTC				Usart1_Putc
#define DCBM1_UART_PUTBFR			Usart1_PutBfr
//#define DCBM1_ERROR_ENABLE

#ifndef USART1_BAUD
#define USART1_BAUD					(2*460800)
#endif
#define USART1_INT_PRIO_RX			configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

#define SMOOTH_MODE_SWITCH			// Enable smooth mode switch

//#define MS_MEASURE				// Enable for measuring without curve and temp compensation
#define MS_TEMPCORR_OFF
//#define MS_TEST_NO_EMITTER			// Enable to enable all control only not from emitter
//#define MS_LOAD_ATW
//#define MS_LOAD_TESTER
//#define MS_LOAD_TW

//#define MS_ENABLE_TEST_SWITCH		// Disable when releasing!!!! User may never be able to write emitter

#define USART2_TX_BFR_SIZE	260
//#define USART2_DMA_CHANNEL	1
//#define USART2_DMA

#if !defined(DEBUG_UART) || DEBUG_UART == 0
#define USART2_BAUD			250000
#define USART2_STOP_BITS	2
#endif

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>
#include <peripheral_clk_config.h>
#include "hal_usart_async.h"
#include "compiler.h"
#include <hal_can_async.h>
#include "rprintf.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "message_buffer.h"
#include "semphr.h"
#include "utils.h"
#include "stdlib.h"
#include "stdio.h"
#include "driver_init.h"
#include "izi_can_frame.h"
#include "misc/debug.h"
#include "logger_data.h"
#include "hal_rand_sync.h"
#include "firmware_info.h"
#include "misc/state.h"
#include "production.h"

typedef enum trace_level_e
{
	TRACE_LEVEL_ERROR	= 0,
	TRACE_LEVEL_WARNING = 1,
	TRACE_LEVEL_INFO = 2,
	TRACE_LEVEL_DEBUG = 3,
	TRACE_LEVEL_VERBOSE = 4,
}trace_level_t;

#define OS_REPORT_ALLTASKS	0x0000000F
typedef enum os_report_task_e
{
	TASK_DEBUG	= 0,
	TASK_CANDRIVER	= 1,
	TASK_STEPDOWN	= 2,
	TASK_IZIPLUS	= 3,
	// When added, adjust OS_REPORT_ALLTASKS
	// Max 32
}os_report_task_t;

extern void ReportStackSize(uint32_t stackSize, uint8_t task_enum);
extern uint32_t os_report_stack;
extern bool os_report_heap;

#define REPORT_STACK(size, task_enum) \
{	\
	if(os_report_stack & (1 << ((uint8_t)task_enum & 0x1F)))	\
	{\
		ReportStackSize(size, (uint8_t)task_enum);	\
		os_report_stack &= ~(1 << ((uint8_t)task_enum & 0x1F));\
	}\
}

#define IZI_DFLT_TRACE_LVL			TRACE_LEVEL_INFO
#define IZI_OUTPUT_DFLT_TRACE_LVL	TRACE_LEVEL_INFO
#define DCBM1_DFLT_TRACE_LVL		TRACE_LEVEL_INFO
#define ITO_DFLT_TRACE_LVL			TRACE_LEVEL_INFO
#define STATE_DFLT_TRACE_LVL		TRACE_LEVEL_INFO
#define IZI_INPUT_DFLT_TRACE_LVL	TRACE_LEVEL_INFO
#define IZI_CAN_TRACE_BUILD_LVL		3	//TRACE_LEVEL_INFO
#define OS_TRACE_BUILD_LVL			2	//TRACE_LEVEL_INFO

#define OS_PREFIX		"[osx]\t"

#ifndef OS_TRACE_BUILD_LVL
#define OS_TRACE_BUILD_LVL	-1
#endif

#if !defined(DEBUG_UART) || DEBUG_UART == 0
#undef OS_TRACE_BUILD_LVL
#define OS_TRACE_BUILD_LVL	-1
#endif

#if OS_TRACE_BUILD_LVL >= 0
#define OS_TRACE_ERROR(...)		{ esp_printf(Debug_Putc, OS_PREFIX, __VA_ARGS__); }
#else
#define OS_TRACE_ERROR(...)
#endif
#if OS_TRACE_BUILD_LVL >= 1
#define OS_TRACE_WARNING(...)	{ esp_printf(Debug_Putc, OS_PREFIX, __VA_ARGS__); }
#else
#define OS_TRACE_WARNING(...)
#endif
#if OS_TRACE_BUILD_LVL >= 2
#define OS_TRACE_INFO(...)	{ esp_printf(Debug_Putc, OS_PREFIX, __VA_ARGS__); }
#define GEN_TRACE_INFO(...)	{ esp_printf(Debug_Putc, "", __VA_ARGS__); }
#else
#define OS_TRACE_INFO(...)
#define GEN_TRACE_INFO(...)
#endif

#define SLAVE_DEVTYPE_GROUP 	DEVTYPE_GROUP_POWERLINE

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
	
#define IZIPLUS_DATAFRAME_ONLY_RXINT	

#endif /* INCLUDES_H_ */
/*
 * includes.h
 *
 * Created: 12-8-2021 21:30:05
 *  Author: Milo
 */ 


#ifndef INCLUDES_H_
#define INCLUDES_H_

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
#include "misc/rprintf.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "driver_init.h"
#include "firmware_info.h"
#include "uart/usart4.h"

typedef enum trace_level_e
{
	TRACE_LEVEL_ERROR	= 0,
	TRACE_LEVEL_WARNING = 1,
	TRACE_LEVEL_INFO = 2,
	TRACE_LEVEL_DEBUG = 3,
	TRACE_LEVEL_VERBOSE = 4,
}trace_level_t;

typedef enum os_report_task_e
{
	TASK_NETWORK	= 0,
	TASK_CANMASTER 	= 1,
	TASK_USBHOST 	= 2,
	TASK_LOCAL_TCP  = 3,
	TASK_DMXIN		= 4,
	TASK_CANDRIVER	= 5,
	TASK_USB_VCOM	= 6,
	TASK_LOGGER		= 7,
	TASK_LOCAL_TCP1 = 8,
	TASK_LOCAL_TCP2 = 9,
	TASK_LOCAL_TCP3 = 10,
	TASK_IZI_TOPIC =  11,
	TASK_OS_TIMER  =  12,
	TASK_OS_IDLE   =  13,
	TASK_DALI	   =  14,
	TASK_HTTP	   =  15,
	TASK_HTTP_SESSION   =  16,
	TASK_SEQUENCE   =  17,
	TASK_SEQUENCE2   =  18,
	TASK_FILESYSTEM   =  19,
	TASK_SEQUENCE3   =  20,
	TASK_SEQUENCE4   =  21,
	TASK_OUTPUT	   =  22,
	// When added, adjust OS_REPORT_ALLTASKS
	// Max 32
}os_report_task_t;

#define IZI_DFLT_TRACE_LVL			TRACE_LEVEL_INFO
#define IZI_OUTPUT_DFLT_TRACE_LVL	TRACE_LEVEL_INFO
#define DCBM1_DFLT_TRACE_LVL		TRACE_LEVEL_INFO
#define ITO_DFLT_TRACE_LVL			TRACE_LEVEL_INFO
#define STATE_DFLT_TRACE_LVL		TRACE_LEVEL_INFO
#define IZI_INPUT_DFLT_TRACE_LVL	TRACE_LEVEL_INFO
#define IZI_CAN_TRACE_BUILD_LVL		3	//TRACE_LEVEL_INFO
#define OS_TRACE_BUILD_LVL			2	//TRACE_LEVEL_INFO
#define IZI_TRACE_BUILD_LVL			TRACE_LEVEL_INFO

#define OS_PREFIX		"[osx]\t"

#ifndef OS_TRACE_BUILD_LVL
#define OS_TRACE_BUILD_LVL	-1
#endif

#if OS_TRACE_BUILD_LVL >= 0
#define OS_TRACE_ERROR(...)		//{ esp_printf(USART4_Putc, OS_PREFIX, __VA_ARGS__); }
#else
#define OS_TRACE_ERROR(...)
#endif
#if OS_TRACE_BUILD_LVL >= 1
#define OS_TRACE_WARNING(...)	//{ esp_printf(USART4_Putc, OS_PREFIX, __VA_ARGS__); }
#else
#define OS_TRACE_WARNING(...)
#endif
#if OS_TRACE_BUILD_LVL >= 2
#define OS_TRACE_INFO(...)	//{ esp_printf(USART4_Putc, OS_PREFIX, __VA_ARGS__); }
#define GEN_TRACE_INFO(...)	//{ esp_printf(USART4_Putc, "", __VA_ARGS__); }
#else
#define OS_TRACE_INFO(...)dd
#define GEN_TRACE_INFO(...)dd
#endif

#define SLAVE_DEVTYPE_GROUP 	DEVTYPE_GROUP_POWERLINE

#endif /* INCLUDES_H_ */
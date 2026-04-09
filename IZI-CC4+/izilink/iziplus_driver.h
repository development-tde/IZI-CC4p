/*
 * iziplus_driver.h
 *
 *  Created on: 22 apr. 2021
 *      Author: Milo
 */

#ifndef IZIPLUS_DRIVER_H_
#define IZIPLUS_DRIVER_H_

#include "includes.h"
#include "izi_module.h"
#include "iziplus_frames.h"

struct iziplus_cmd_delayed_req_s {
	uint8_t cmd;
	uint8_t state;
	uint16_t time;
	uint32_t length;
	uint8_t tokenresult;
	iziplus_msgtype_u msg_type;
	uint32_t address;
	uint8_t data[256];
} __attribute__((packed));

typedef struct iziplus_cmd_delayed_req_s iziplus_cmd_delayed_req_t;

typedef struct iziplus_nwtemp_s {
	// General
	uint32_t pan_id;
	uint8_t pl_frequency;
	uint8_t pl_rate;
	uint8_t short_id;
	uint8_t pl_txlevel;
} iziplus_nwtemp_t;

#define IZI_PREFIX	"[izi]\t"

#ifndef  IZI_TRACE_BUILD_LVL
#define IZI_TRACE_BUILD_LVL		2
#endif

#if !defined(DEBUG_UART) || DEBUG_UART == 0
#undef IZI_TRACE_BUILD_LVL
#define IZI_TRACE_BUILD_LVL -1
#endif

#if IZI_TRACE_BUILD_LVL >= 0
#define IZI_TRACE_ERROR(...)	{ if(izi_trace_lvl >= TRACE_LEVEL_ERROR)		{ esp_printf(Debug_Putc, IZI_PREFIX, __VA_ARGS__); } }
#else
#define IZI_TRACE_ERROR(...)
#endif
#if IZI_TRACE_BUILD_LVL >= 1
#define IZI_TRACE_WARNING(...)	{ if(izi_trace_lvl >= TRACE_LEVEL_WARNING)		{ esp_printf(Debug_Putc, IZI_PREFIX, __VA_ARGS__); } }
#else
#define IZI_TRACE_WARNING(...)
#endif
#if IZI_TRACE_BUILD_LVL >= 2
#define IZI_TRACE_INFO(...)		{ if(izi_trace_lvl >= TRACE_LEVEL_INFO)		{ esp_printf(Debug_Putc, IZI_PREFIX, __VA_ARGS__); } }
#else
#define IZI_TRACE_INFO(...)
#endif
#if IZI_TRACE_BUILD_LVL >= 3
#define IZI_TRACE_DEBUG(...)	{ if(izi_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, IZI_PREFIX, __VA_ARGS__); } }
#else
#define IZI_TRACE_DEBUG(...)
#endif

#include "logger_data.h"

#define LOGWRITE_CRITICAL(errcode, text, log_source)	//izican_log_write(errcode, text, log_critical, 0, log_source)
#define LOGWRITE_ERROR(errcode, text, log_source)		//izican_log_write(errcode, text, log_error, 0, log_source)
#define LOGWRITE_WARNING(errcode, text, log_source)		//izican_log_write(errcode, text, log_warning, 0, log_source)
#define LOGWRITE_INFO(errcode, text, log_source)		//izican_log_write(errcode, text, log_info, 0, log_source)
#define LOGWRITE_TRACE(errcode, text, log_source)		//izican_log_write(errcode, text, log_trace, 0, log_source)

#define LOGWRITE_CRITICAL_S(errcode, text, log_source, serial)		//izican_log_write(errcode, text, log_critical, serial, log_source)
#define LOGWRITE_ERROR_S(errcode, text, log_source, serial)		//izican_log_write(errcode, text, log_error, serial, log_source)
#define LOGWRITE_WARNING_S(errcode, text, log_source, serial)		//izican_log_write(errcode, text, log_warning, serial, log_source)
#define LOGWRITE_INFO_S(errcode, text, log_source, serial)		//izican_log_write(errcode, text, log_info, serial, log_source)

void IziPlus_Init();
void IziPlus_Debug(uint8_t *data, uint8_t length);
void IziPlus_ContactReport(uint8_t contact_flag);
bool IziPlus_LightDataOk();
void IziPlus_IdentifyLocal();

#endif /* IZIPLUS_DRIVER_H_ */

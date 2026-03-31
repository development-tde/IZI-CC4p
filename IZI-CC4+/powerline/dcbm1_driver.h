/*
 * dcbm1_driver.h
 *
 *  Created on: 4 sep. 2020
 *      Author: Milo
 */

#ifndef DCBM1_DRIVER_H_
#define DCBM1_DRIVER_H_

typedef void (*dcbm1_cb_func_rxptr)(unsigned char c);
typedef void (*dcbm1_cb_func_txptr)(void);

#define DCBM1

#define DCBM1_PREFIX	"[dcb]\t"

#if !defined(DEBUG_UART) || DEBUG_UART == 0
#undef DCBM1_TRACE_ENABLE
#endif

#ifdef DCBM1_TRACE_ENABLE

#define DCBM1_TRACE_ERROR(...)		if(dcbm1_trace_lvl >= TRACE_LEVEL_ERROR)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_WARNING(...)	if(dcbm1_trace_lvl >= TRACE_LEVEL_WARNING)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_INFO(...)		if(dcbm1_trace_lvl >= TRACE_LEVEL_INFO)			{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_DEBUG(...)		if(dcbm1_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
	
#else

#define DCBM1_TRACE_ERROR(...)		//if(dcbm1_trace_lvl >= TRACE_LEVEL_ERROR)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_WARNING(...)	//if(dcbm1_trace_lvl >= TRACE_LEVEL_WARNING)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_INFO(...)		//if(dcbm1_trace_lvl >= TRACE_LEVEL_INFO)			{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }
#define DCBM1_TRACE_DEBUG(...)		//if(dcbm1_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, DCBM1_PREFIX, __VA_ARGS__); }

#endif // DCBM1_TRACE_ENABLE

void dcbm1_driver_init(void);
void dcbm1_debug(uint8_t *data, uint8_t length);
void dcbm1_set_rx_callback(dcbm1_cb_func_rxptr ptr);
void dcbm1_set_txready_callback(dcbm1_cb_func_txptr ptr);
bool dcbm1_writereg(uint8_t reg, uint8_t data);
uint16_t dcbm1_send(volatile uint8_t *bfr, uint16_t length);
uint16_t dcbm1_send_nb(volatile uint8_t *bfr, uint16_t length);
void dcbm1_reset();
int8_t dcbm1_init(uint8_t frequency, uint8_t rate, uint8_t txlevel);
int8_t dcbm1_setfrequency(uint8_t frequency);
bool dcbm1_checkfrequency(uint8_t frequency);
int8_t dcbm1_setrate(uint8_t rate);
bool dcbm1_checkrate(uint8_t rate);
int dcbm1_getfrequency_h(uint8_t frequency);
int dcbm1_getfrequency_l(uint8_t frequency);
int dcbm1_getrate_kHz(uint8_t rate);
void dcbm1_checkerror();
void dcbm1_enable_bus_busy(bool enable);
bool dcbm1_sleep();
void dcbm1_wake();
int8_t dcbm1_settxlevelraw(uint8_t txlevel);

#endif /* DCBM1_DRIVER_H_ */

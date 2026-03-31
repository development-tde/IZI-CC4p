/*
 * izi_input.h
 *
 *  Created on: 8 apr. 2021
 *      Author: Milo
 */

#ifndef IZI_INPUT_H_
#define IZI_INPUT_H_


#define IZI_TIMEROS_TICKS	25

#define IZI_INP_PREFIX	"[inp]\t"

#if DEBUG_UART > 0
#define IZI_INPUT_TRACE_ERROR(...)		if(izi_input_trace_lvl >= TRACE_LEVEL_ERROR)		{ esp_printf(Debug_Putc, IZI_INP_PREFIX, __VA_ARGS__); }
#define IZI_INPUT_TRACE_WARNING(...)	if(izi_input_trace_lvl >= TRACE_LEVEL_WARNING)		{ esp_printf(Debug_Putc, IZI_INP_PREFIX, __VA_ARGS__); }
#define IZI_INPUT_TRACE_INFO(...)		if(izi_input_trace_lvl >= TRACE_LEVEL_INFO)			{ esp_printf(Debug_Putc, IZI_INP_PREFIX, __VA_ARGS__); }
#define IZI_INPUT_TRACE_DEBUG(...)		if(izi_input_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, IZI_INP_PREFIX, __VA_ARGS__); }
#else
#define IZI_INPUT_TRACE_ERROR(...)		
#define IZI_INPUT_TRACE_WARNING(...)	
#define IZI_INPUT_TRACE_INFO(...)		
#define IZI_INPUT_TRACE_DEBUG(...)			
#endif

// Proto
void IziInput_Init();
void IziInput_10ms();

#endif /* IZI_INPUT_H_ */

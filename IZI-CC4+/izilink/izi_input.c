/*
 * izi_input.c
 *
 *  Created on: 8 apr. 2021
 *      Author: Milo
 */

#include "includes.h"
#include "izi_input.h"
#include "appconfig.h"
#include "iziplus_module_def.h"
#include "iziplus_driver.h"
#include "stepdown.h"

// Defines
#define TIME_SEC					100
#define TIME_START_IDENTIFY			1			// In seconds
#define TIME_WINDOW_IDENTIFY		2
#define TIME_START_FACTORY_DFLT		10
#define TIME_START_LOG_DFLT			5
#define TIME_WINDOW_FACTORY_DFLT	2
#define TIME_WINDOW_LOG_DFLT		2
#define TIME_CLICK					(TIME_SEC/2)
#define TIME_CLIP_DEBOUNCE			2400		// 40 * 60 = 1 minute
#define TIME_HANDLED_DEBOUNCE		(TIME_CLIP_DEBOUNCE + 1)

#define EXT_DEBOUNCE				(TIME_SEC/25)		// 40ms debounce for external input (in practice this will be between 30ms and 40ms)

// Globals
trace_level_t izi_input_trace_lvl = IZI_INPUT_DFLT_TRACE_LVL;

static uint16_t iziinput_switchdbc = 0;
static uint16_t iziinput_switchoffdbc = 0;
static uint16_t iziinput_switchclicks = 0;
static bool iziinput_switchactive = false;
static char text[LOG_TEXT_SIZE + 8];

static uint16_t iziinput_extdbc = 0;
static bool iziinput_extactive = false;
static uint16_t iziinput_extoffdbc = 0;


static uint8_t iziinput_level, iziinput_sw_mode = 0, iziinput_sw_select;
static uint8_t iziinput_logall_dflt_timer = 0;

TimerHandle_t  xIziInput_Timer = NULL;

// Proto

// Code

void IziInput_Init()
{
	/*if(xIziInput_Timer == NULL)
		xIziInput_Timer = xTimerCreate("Izi_InputTmr", IZI_TIMEROS_TICKS, pdTRUE, ( void * ) 0, IziInput_25ms);

	if(xIziInput_Timer == NULL)
	{
		IZI_INPUT_TRACE_INFO("Timer not created Izi preset");	// Todo: what to do
	}
	else if( xTimerStart(xIziInput_Timer, 0) != pdPASS )
	{
		IZI_INPUT_TRACE_INFO("Timer not started Izi preset\r\n");				// Todo: what to do
	}*/
}

void IziInput_10ms(TimerHandle_t xTimer)
{
	if(iziinput_logall_dflt_timer)
		iziinput_logall_dflt_timer--;
		
	if(gpio_get_pin_level(SWITCH) == 0) 
	{
		if(iziinput_switchdbc < TIME_CLIP_DEBOUNCE)	// Clip at on minute
			iziinput_switchdbc++;
		if(iziinput_switchdbc == TIME_SEC/10)		// 100ms active
		{
			iziinput_switchactive = true;
			IZI_INPUT_TRACE_INFO("Switch active\r\n");
		}
		else if(iziinput_switchdbc == TIME_START_IDENTIFY*TIME_SEC)		// All active off option
			State_SetAttentionInfo(COLOR_MAGENTA, COLOR_BLUE, TIME_WINDOW_IDENTIFY, 2);
		else if(iziinput_switchdbc == TIME_START_FACTORY_DFLT*TIME_SEC)	// Factory default
			State_SetAttentionInfo(COLOR_WHITE, COLOR_RED, TIME_WINDOW_FACTORY_DFLT, 2);
		else if(iziinput_switchdbc == TIME_START_LOG_DFLT*TIME_SEC)	// Log default
			State_SetAttentionInfo(COLOR_GREEN, COLOR_MAGENTA, TIME_WINDOW_FACTORY_DFLT, 2);

		iziinput_switchoffdbc = 0;
	}
	else if(++iziinput_switchoffdbc >= 2 && iziinput_switchactive)		// Accept one low before resetting debounce, and handle switch functions
	{
		if(iziinput_switchdbc > TIME_CLICK)
		{
			if(iziinput_switchdbc < TIME_HANDLED_DEBOUNCE)
			{
				sprintf(text, "Switch active (press time %d ms)", iziinput_switchdbc * (1000/TIME_SEC));
			}
			else
				sprintf(text, "Switch active and reported handled");
			LOGWRITE_INFO(ERRCODE_SWITCH_ACTIVE, text, log_src_task_postpone);
		}
		else
			iziinput_switchclicks++;

		if((iziinput_switchdbc >= (TIME_START_IDENTIFY*TIME_SEC)) && (iziinput_switchdbc <= ((TIME_START_IDENTIFY + TIME_WINDOW_IDENTIFY)*TIME_SEC)))
		{
			IziPlus_IdentifyLocal();
		}
		else if((iziinput_switchdbc >= (TIME_START_LOG_DFLT*TIME_SEC)) && (iziinput_switchdbc <= ((TIME_START_LOG_DFLT + TIME_WINDOW_LOG_DFLT)*TIME_SEC)))
		{
			AppLog_Default(false);
			iziinput_logall_dflt_timer = 150;			// 1.5 sec window to clear operation times by an extra click
		}
		else if((iziinput_switchdbc >= (TIME_START_FACTORY_DFLT*TIME_SEC)) && (iziinput_switchdbc <= ((TIME_START_FACTORY_DFLT + TIME_WINDOW_FACTORY_DFLT)*TIME_SEC)))
		{
			AppConfig_Default();
			AppLog_Default(false);
		}
		iziinput_switchactive = false;
		iziinput_switchdbc = 0;
	}
	else if(!iziinput_switchactive)
	{
		iziinput_switchdbc = 0;
		if(iziinput_switchoffdbc < (TIME_SEC*60))
			iziinput_switchoffdbc++;

		if(iziinput_switchoffdbc > (TIME_SEC/2))
		{
			if(iziinput_switchclicks > 0)
			{
				State_ResetToggleData_Show();
				if(iziinput_switchclicks == 3)
				{
					iziinput_sw_mode = iziinput_sw_mode ? 0 : 1;
					State_SetAttentionInfo(COLOR_WHITE, COLOR_YELLOW, 4, 2);
					
					iziinput_level = 0;
					IziPlus_Module_Overrule_Ch1234(iziinput_level, 10*60*1000, PRIO_OVERRULE_MEDIUM);		// 10 minute on (or if off, free overrule)
					
					State_SetAttentionInfo(COLOR_WHITE, COLOR_YELLOW, 10*60, 2);
				}
				else if(iziinput_switchclicks == 2)
				{
				}
				else 
				{
					if(iziinput_logall_dflt_timer)				// Only default, just after 5 sec hold default
					{
						AppLog_Default(true);
					}
					//Do things on 1 click
					else if(iziinput_sw_mode)
					{
						if(iziinput_level == 255)
							iziinput_level = 0;
						else
							iziinput_level += 25;				// Per 10%
						
						if(iziinput_level == 250)
							iziinput_level = 255;
						
						for(int i = 0; i < 4; i++)
							IziPlus_Module_Overrule_Chx(i, iziinput_level, 10*60*1000, PRIO_OVERRULE_MEDIUM);		// 10 minute on (or if off, free overrule)
						
						State_SetAttentionInfo(COLOR_WHITE, COLOR_YELLOW, 10*60, 2);
					}
					else
					{
						if(!IziPlus_Module_IsOverruled())				// Both not running, begin with channel one again
							iziinput_sw_select = 0;
						
						if(IziPlus_Module_Test(iziinput_sw_select, 5000, PRIO_OVERRULE_MEDIUM))
							iziinput_sw_select = 0;
						else
							iziinput_sw_select++;
					}
				}
				sprintf(text, "Switch active %d clicks", iziinput_switchclicks);
				LOGWRITE_INFO(ERRCODE_SWITCH_ACTIVE, text, log_src_task_postpone);
				iziinput_switchclicks = 0;
			}
		}
	}
	
	if(gpio_get_pin_level(EXT) == 0)
	{
		if(iziinput_extdbc < TIME_CLIP_DEBOUNCE)	// Clip at on minute
			iziinput_extdbc++;
		if(iziinput_extdbc == EXT_DEBOUNCE)		// 40ms active
		{
			IziPlus_ContactReport(1);
			iziinput_extactive = true;
			iziinput_extoffdbc = 0;
			IZI_INPUT_TRACE_INFO("Ext input active\r\n");
			State_SetAttentionInfo(COLOR_GREEN, COLOR_BLUE, 2, 2);
		}
	}
	else if(++iziinput_extoffdbc >= 2 && iziinput_extactive)		// Accept one low before resetting debounce, and handle ext functions
	{
		iziinput_extdbc = 0;
		iziinput_extactive = false;
		IziPlus_ContactReport(0);
		IZI_INPUT_TRACE_INFO("Ext input inactive\r\n");
	}
	else if(!iziinput_extactive)
		iziinput_extdbc = 0;
}

uint8_t IziInput_GetExtInput()
{
	return iziinput_extactive ? 1 : 0;
}

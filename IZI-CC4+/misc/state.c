/*
 * state.c
 *
 *  Created on: 13 feb. 2021
 *      Author: Milo
 */


#include "includes.h"
#include "state.h"
#include "hal_pwm.h"
#include "hpl_tcc.h"
#include "hpl_tc_base.h"
#include "hal_timer.h"
#include "peripheral_clk_config.h"
#include "hpl_tcc_config.h"
//#include "timer.h"
//#include "timers.h"
//#include "izi_input.h"

// Defines
#define STATE_TIMERFADE_INTERVAL		2		//ms
#define CAN_TIMERFADE_INTERVAL			2		//ms
#define STATE_TIMER_COUNT_PRS			20				// 5Hz / 20 = 0.25Hz

#define STATE_TIMER_FREQ			200				// 200Hz

#define STATE_TIMEROS_TICKS			200

// Globals
static volatile uint32_t state_errors = 0, state_error_blink = 0;
static volatile uint32_t state_warnings = 0, state_warning_blink = 0;
static volatile uint32_t state_startinfo = 0;
static volatile uint32_t state_runinfo = 0;

static volatile uint8_t state_attentioncolor1 = 0;
static volatile uint8_t state_attentioncolor2 = 0;
static volatile uint8_t state_attentionspeed = 0;
static volatile uint8_t state_attentionstate = 0;
static volatile uint16_t state_attentiontime = 0;
static volatile uint16_t state_attentionprs = 0;

static volatile uint16_t state_prs = 0;
static volatile uint16_t state_code = 0;
static volatile uint16_t state_code_exe = 0;
static volatile uint16_t state_phase = 0, state_counter = 0;
static volatile uint16_t state_delay = 0;
static volatile bool state_correction = false;
static volatile uint16_t state_syncwait = 0;

static volatile uint16_t state_fade[3] = { 0,0,0 };
static volatile uint16_t state_fade_end[3] = { 0,0,0 };
static volatile uint16_t state_fade_delta[3] = { 1,1,1 };

static volatile bool state_free_toggle;
static volatile uint8_t state_toggle;
static volatile uint16_t state_toggle_timeout;

static volatile uint16_t state_com_show = 5 * 60;		// 1 minute

//struct pwm_descriptor	PWM_0;
//struct pwm_descriptor	PWM_1;
//struct timer_descriptor TIMER_0;
//static struct timer_task TIMER_0_task1, TIMER_0_task2;

trace_level_t state_trace_lvl = STATE_DFLT_TRACE_LVL;

const uint8_t color_table[][3] =
{
	{ 100, 0, 0},			// COLOR_RED
	{ 0, 100, 0},			// COLOR_GREEN
	{ 0, 0, 100},			// COLOR_BLUE
	{ 100, 60, 0},			// COLOR_YELLOW
	{ 100, 0, 100},			// COLOR_MAGENTA
	{ 0, 100, 100},			// COLOR_AQUA
	{ 100, 100, 100},		// COLOR_WHITE
	{ 100, 20, 0},			// COLOR_ORANGE
	{ 100, 30, 60},			// COLOR_PINK
	{ 0, 0, 0},				// COLOR_OFF
	{ 30, 18, 0},			// COLOR_YELLOW_LOW
	{ 0, 0, 10},			// COLOR_BLUE_LOW
	{ 0, 20, 0},			// COLOR_GREEN_LOW
	{ 20, 0, 0},			// COLOR_RED_LOW
	{ 20, 20, 20},			// COLOR_WHITE_LOW
	{ 40, 40, 100},			// COLOR_LIGHT_BLUE
	{ 0, 60, 60},			// COLOR_AQUA_MEDIUM
};

#define PWM_PERIOD_HZ		4000
#define PWM_FREQ			((CONF_GCLK_TCC0_FREQUENCY / (1 << CONF_TCC0_PRESCALER)) / PWM_PERIOD_HZ)
#define PWM_DUTY_PERCENT(p)	(p * PWM_FREQ)/100
#define PWM_DUTY_PERCENT100(p)	(p * PWM_FREQ)/10000


#define STATE_MAT_R		3
#define STATE_MAT_G		0
#define STATE_MAT_B		2
#define STATE_PERIOD_R	2
#define STATE_PERIOD_G	3
#define STATE_PERIOD_B	3

// Proto
void State_SetLed(uint8_t color_idx, uint16_t fade_ms);

char *StateErrorText[STATE_ERROR_DEFINED] =
{
	"No production data",			// STATE_ERROR_NO_SERIAL
	"Hardware error",				// STATE_ERROR_HW_ERR
	"No communication",				// STATE_ERROR_NO_COM
	"Vin too low",					// STATE_ERROR_VIN_LOW
	"Output 1 short",				// STATE_ERROR_OUTPUT1_SHORT
	"Output 2 short",				// STATE_ERROR_OUTPUT2_SHORT
	"Output 3 short",				// STATE_ERROR_OUTPUT3_SHORT
	"Output 4 short",				// STATE_ERROR_OUTPUT4_SHORT
	"NTC error",					// STATE_ERROR_NTC
	"Incomplete com"				// STATE_ERROR_INCOMPLETE_COM
};

char *StateWarningText[STATE_WARNING_DEFINED] =
{
	"Output 1 open",				// STATE_WARNING_OUTPUT1_OPEN
	"Output 2 open",				// STATE_WARNING_OUTPUT2_OPEN
	"Output 3 open",				// STATE_WARNING_OUTPUT3_OPEN
	"Output 4 open",				// STATE_WARNING_OUTPUT4_OPEN
	"Reboot needed",				// STATE_WARNING_REBOOT_NEEDED
	"Bad com quality",				// STATE_WARNING_BAD_COMQUAL
	"Supply too low",				// STATE_WARNING_VIN_LOW
	"Intern temperature high",		// STATE_WARNING_TEMP_HIGH
	"NTC1 temperature too high",	// STATE_WARNING_NTC1_HIGH
	"NTC2 temperature too high",	// STATE_WARNING_NTC2_HIGH
	"Total power too high",			// STATE_WARNING_POWER_TOT_HIGH
	"Supply voltage too high",		// STATE_WARNING_SUPPLY_HIGH
	"Timing warning",				// STATE_WARNING_TIMING_WARN
	"Uart overflow",				// STATE_WARNING_UART_OVW_WARN
	"DMX warning",					// STATE_WARNING_DMX_WARN
	"",
	"Power limit Ch1",				// STATE_WARNING_POWER1_HIGH
	"Power limit Ch2",				// STATE_WARNING_POWER2_HIGH
	"Power limit Ch3",				// STATE_WARNING_POWER3_HIGH
	"Power limit Ch4",				// STATE_WARNING_POWER4_HIGH
};

const char *StateStartInfoText[STATE_STARTINFO_DEFINED] =
{
	"Start CAN bus",				// STATE_START_CANBUS
	"Start IZI+ bus",				// STATE_START_IZILINK
	"Starting OS"			,		// STATE_START_MAIN
};

// Proto
void State_Timer200ms(void);


void State_SetError(uint8_t error)
{
	if(error < STATE_ERROR_MAX)
	{
		if(state_errors == 0)
			state_phase = 0;
		state_errors |= (1 << error);
	}
}

void State_ClearError(uint8_t error)
{
	if(error < STATE_ERROR_MAX)
	{
		state_errors &= ~(1 << error);
	}
}

bool State_IsErrorActive(uint8_t error)
{
	return state_errors & (1 << error);
}

uint32_t State_GetErrors()
{
	return state_errors;
}

int8_t State_GetHighestError()
{
	for(uint8_t i = 0; i < STATE_ERROR_MAX; i++)
	{
		if(state_errors & (1 << i))			// Lowest bit is most important error
			return i;
	}
	return 0;
}

char* State_GetHighestErrorText()
{
	int8_t error = State_GetHighestError();
	if(error >= 0)
	{
		if(error < STATE_ERROR_DEFINED)
			return (char*)StateErrorText[error];
		return "Unknown error";
	}
	return "";
}

void State_SetWarning(uint8_t warning)
{
	if(warning < STATE_WARNING_MAX)
	{
		if(state_errors == 0 && state_warnings == 0)
		{
			state_phase = 0;
			state_delay = 0;
		}
		state_warnings |= (1 << warning);
	}
}

void State_ClearWarning(uint8_t warning)
{
	if(warning < STATE_WARNING_MAX)
	{
		bool wasNotZero = state_warnings > 0;
		state_warnings &= ~(1 << warning);
		if(wasNotZero && state_warnings == 0)
			state_syncwait = 5 * 5;				// Max wait for sync is 5 sec
	}
}

bool State_IsWarningActive(uint8_t warning)
{
	return state_warnings & (1 << warning);
}

uint32_t State_GetWarnings()
{
	return state_warnings;
}

int8_t State_GetHighestWarning()
{
	for(uint8_t i = 0; i < STATE_WARNING_MAX; i++)
	{
		if(state_warnings & (1 << i))			// Lowest bit is most important error
			return i;
	}
	return -1;
}

char* State_GetHighestWarningText()
{
	int8_t warning = State_GetHighestWarning();
	if(warning >= 0)
	{
		if(warning < STATE_WARNING_DEFINED)
			return (char*)StateWarningText[warning];
		return "Unknown warning";
	}
	return "";
}

void State_SetStartInfo(uint8_t startinfo)
{
	if(startinfo < STATE_STARTINFO_MAX)
	{
		state_startinfo |= (1 << startinfo);
	}
}

void State_ClearStartInfo(uint8_t startinfo)
{
	if(startinfo < STATE_STARTINFO_MAX)
	{
		bool wasNotZero = state_startinfo > 0;
		state_startinfo &= ~(1 << startinfo);
		if(wasNotZero && state_startinfo == 0)
			state_syncwait = 5 * 5;				// Max wait for sync is 5 sec
	}
}

uint32_t State_GetStartInfo()
{
	return state_startinfo; 
}

uint8_t State_GetHighestStartInfo()
{
	for(uint8_t i = 0; i < STATE_STARTINFO_MAX; i++)
	{
		if(state_startinfo & (1 << i))			// Lowest bit is most important error
			return i;
	}
	return -1;
}

char* State_GetHighestStartInfoText()
{
	int8_t startinfo = State_GetHighestStartInfo();
	if(startinfo >= 0)
	{
		if(startinfo < STATE_STARTINFO_DEFINED)
			return (char*)StateStartInfoText[startinfo];
		return "Unknown init state";
	}
	return "";
}

void State_SetRunInfo(uint8_t runinfo)
{
	if(runinfo < STATE_RUNINFO_MAX)
	{
		state_runinfo |= (1 << runinfo);
	}
}

void State_ClearRunInfo(uint8_t runinfo)
{
	if(runinfo < STATE_RUNINFO_MAX)
	{
		bool wasNotZero = state_runinfo > 0;
		state_runinfo &= ~(1 << runinfo);
		if(wasNotZero && state_runinfo == 0)
			state_syncwait = 5 * 5;				// Max wait for sync is 5 sec
	}
}

uint32_t State_GetRunInfo()
{
	return state_runinfo;
}

/**
 * Blink between color_idx1 and color_idx2 for time in seconds. Speed is a prescaler for the blink speed
 */
void State_SetAttentionInfo(uint8_t color_idx1, uint8_t color_idx2, uint16_t time, uint8_t speed)
{
	state_attentioncolor1 = color_idx1;
	state_attentioncolor2 = color_idx2;
	state_attentiontime = time*5;
	state_attentionspeed = speed;
	if(state_attentiontime == 0)
	{
		state_attentionprs = 0;
		state_attentionstate = 0;
	}
}

bool State_IsAttentionBusy()
{
	return state_attentiontime > 0;
}

void State_ClearAttentionInfo()
{
	state_attentiontime = 0;
}

void State_debug(uint8_t *data, uint8_t length)
{
	if(data[0] == 't' && length >= 2)
	{
		state_trace_lvl = data[1] - '0';
		STATE_TRACE_ERROR("Trace level: %d\r\n", state_trace_lvl);
	}
	else if(data[0] == 's' && length >= 3)
	{
		if(data[1] == 'e')
		{
			if(data[2] == 'c')
				State_ClearError(data[3] - '0');
			else if(data[2] == 's')
				State_SetError(data[3] - '0');
		}
		else if(data[1] == 'w')
		{
			if(data[2] == 'c')
				State_ClearWarning(data[3] - '0');
			else if(data[2] == 's')
				State_SetWarning(data[3] - '0');
		}
	}
	else if(data[0] == 'l' && length >= 3)
	{
		State_SetLed(data[1] - '0', (data[2] - '0') * 100);
	}
}

void State_SetLed(uint8_t color_idx, uint16_t fade_ms)
{
	if(fade_ms > 0)
	{
		state_fade_delta[0] = abs(((int16_t)color_table[color_idx][0] * 100) - (int16_t)state_fade[0]) / (fade_ms / STATE_TIMERFADE_INTERVAL);
		if(state_fade_delta[0] == 0)
			state_fade_delta[0] = 1;
		state_fade_delta[1] = abs(((int16_t)color_table[color_idx][1] * 100) - (int16_t)state_fade[1]) / (fade_ms / STATE_TIMERFADE_INTERVAL);
		if(state_fade_delta[1] == 0)
			state_fade_delta[1] = 1;
		state_fade_delta[2] = abs(((int16_t)color_table[color_idx][2] * 100) - (int16_t)state_fade[2]) / (fade_ms / STATE_TIMERFADE_INTERVAL);
		if(state_fade_delta[2] == 0)
			state_fade_delta[2] = 1;
	}
	else
	{
		state_fade_delta[0] = 0;
		state_fade_delta[1] = 0;
		state_fade_delta[2] = 0;
	}
	state_fade_end[0] = color_table[color_idx][0] * 100;
	state_fade_end[1] = color_table[color_idx][1] * 100;
	state_fade_end[2] = color_table[color_idx][2] * 100;
}

void State_ClrLed(uint16_t fade_ms)
{
	State_SetLed(COLOR_OFF, fade_ms);
}


void State_Blink(uint8_t color_idx, uint32_t flags)
{
	if(state_phase == 0)
	{
		for(uint8_t i = 0; i < STATE_ERROR_MAX; i++)
		{
			if(flags & (1 << i))			// Lowest bit is most important error
			{
				state_code = i + 1;
				break;
			}
		}
		state_phase = 1;
		state_prs = 0;
		state_code_exe = 0;
		State_ClrLed(50);
	}
	else if(state_phase == 1)
	{
		state_phase = 2;			// Extra delay before code is shown
	}
	else if(state_phase == 2)
	{
		if(++state_prs & 0x01)
		{
			State_ClrLed(50);
			if(state_code_exe++ >= state_code)
			{
				state_phase = 3;
				state_prs = 0;
			}
		}
		else
			State_SetLed(color_idx, 50);
	}
	else if(state_phase == 4)
	{
		State_ClrLed(50);
		if(++state_prs > 3)		// 0.8s nothing
			state_phase = 0;
	}
	else
		state_phase = 0;
}

#define IZI_LED_RED		3	// WO[3]	TCC0
#define IZI_LED_GREEN	1	// WO[1]
#define IZI_LED_BLUE	0	// WO[0]

static volatile uint8_t state_init_led = 0;
static volatile uint16_t tc4_prs = 0;

void TC4_Handler(void)
{
	TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;				// Clear
	
	if(++tc4_prs >= 100)
	{
		State_Timer200ms();
		tc4_prs = 0;
	}
	
	uint8_t i;
		
	for(i = 0; i < 3; i++)
	{
		if(state_fade[i] > state_fade_end[i])
		{
			if((state_fade_delta[i] < state_fade[i]) && state_fade_delta[i] > 0)
				state_fade[i] -= state_fade_delta[i];
			else
				state_fade[i] = state_fade_end[i];
		}
		else if(state_fade[i] < state_fade_end[i])
		{
			if(((state_fade[i] + state_fade_delta[i]) <= state_fade_end[i]) && state_fade_delta[i] > 0)
				state_fade[i] += state_fade_delta[i];
			else
				state_fade[i] = state_fade_end[i];
		}
		else if(state_init_led == 0)
			continue;

		if(i == 0)
			gpio_set_pin_level(LED_R, state_fade[i] == 0);//_pwm_setduty(TCC0, IZI_LED_RED, PWM_DUTY_PERCENT100(state_fade[i]));
		else if(i == 1)
			gpio_set_pin_level(LED_G, state_fade[i] == 0);//_pwm_setduty(TCC0, IZI_LED_GREEN, PWM_DUTY_PERCENT100(state_fade[i]));
		else
			gpio_set_pin_level(LED_B, state_fade[i] == 0);//_pwm_setduty(TCC0, IZI_LED_BLUE, PWM_DUTY_PERCENT100(state_fade[i]));
	}
}

void State_Timer200ms(void)
{
	if(++state_counter >= STATE_TIMER_COUNT_PRS)
		state_counter = 0;
	
	if(state_counter == 0 && state_delay == 1)
	{
		state_delay = 0;
		state_phase = 0;
	}
	
	if(state_com_show > 0)
	{
		if(--state_com_show == 0)
		{
			if(state_free_toggle)
			{
				State_SetLed(COLOR_OFF, 0);
			}
		}
	}
	
	state_free_toggle = false;
	
	if(state_attentiontime && ((state_warnings == 0 && state_errors == 0) || state_delay > 0))
	{
		if(++state_attentionprs >= state_attentionspeed)
		{
			if(!state_attentionstate)
			{
				State_SetLed(state_attentioncolor1, 50);
				state_attentionstate = 1;
			}
			else
			{
				State_SetLed(state_attentioncolor2, 50);
				state_attentionstate = 0;
			}
			state_attentionprs = 0;
		}
		state_attentiontime--;
		state_phase = 0;
		if(state_delay > 1)
			state_delay--;
		return;
	}
	if((state_errors > 0 || state_error_blink > 0) && state_delay == 0)
	{
		if(state_error_blink == 0)										// Only change when previous blink pattern finished
			state_error_blink = state_errors;							// Copy in case it is removed during displaying the pattern
		
		State_Blink(COLOR_RED, state_error_blink);
		if(state_phase == 0)
		{
			state_delay = state_attentiontime > 10 ? 21 : 1;			// When attention time is active extend the wait time, so the attention is not continuously disturbed by an error or warning.
			state_error_blink = 0;										// The start of the error/warning display is done is by the state_counter, which is synced via the IZI bus heartbeat
		}
	}
	else if((state_warnings > 0 || state_warning_blink > 0) && state_delay == 0)
	{
		if(state_warning_blink == 0)									// Only change when previous blink pattern finished
			state_warning_blink = state_warnings;						// Copy in case it is removed during displaying the pattern
		
		State_Blink(COLOR_YELLOW, state_warning_blink);
		if(state_phase == 0)
		{
			state_delay = state_attentiontime > 10 ? 21 : 1;			// When attention time is active extend the wait time, so the attention is not continuously disturbed by an error or warning.
			state_warning_blink = 0;									// The start of the error/warning display is done is by the state_counter, which is synced via the IZI bus sync bit
		}
	}				
	else if(state_startinfo > 0 || state_syncwait)
	{
		if(state_phase > 0)
		{
			state_phase = 0;
			State_SetLed(COLOR_BLUE, 50);			// Blink while starting
		}
		else
		{
			state_phase = 1;
			State_ClrLed(50);
		}
		if(state_syncwait)
			state_syncwait--;
	}
	else if(state_runinfo > 0)
	{
		if((state_phase == 1 || state_phase == 2 || state_phase == 4 || state_phase == 5) && (state_runinfo & (1 << STATE_RUN_DISCOVER)))
		{
			State_SetLed(COLOR_LIGHT_BLUE, 400);
		}
		else if((state_phase == 7 || state_phase == 8 || state_phase == 10 || state_phase == 11) && (state_runinfo & (1 << STATE_RUN_CONFIG_SET)))
		{
			State_SetLed(COLOR_AQUA, 400);
		}
		/*else if((state_phase == 13 || state_phase == 14 || state_phase == 16 || state_phase == 17) && (state_runinfo & ((1 << STATE_RUN_TCPSOCKET)|(1 << STATE_RUN_TCPSOCKET2)|(1 << STATE_RUN_TCPSOCKET3))))
		{
			State_SetLed(COLOR_AQUA, 400);
		}*/
		else
		{
			State_SetLed(COLOR_GREEN_LOW, 200);
		}
		if(++state_phase >= 20)
			state_phase = 0;
	}
	else
	{
		state_free_toggle = true;
		if(state_toggle_timeout > 0)
		{
			if(--state_toggle_timeout == 0)
				State_SetLed(COLOR_OFF, 200);
		}
		state_phase = 0;
	}

	if(state_delay > 1)
		state_delay--;
	
	if(state_toggle_timeout > 0)
	{
		if(--state_toggle_timeout == 0)
			State_SetLed(COLOR_OFF, 200);
	}
}

/**
 * Called every 4 seconds on receive of heartbeat with a time dividable by 4
 */
void State_TimerSync()
{
	if(state_attentiontime)
		;
	else if(state_errors > 0)
		;
	else if(state_warnings > 0)
	{
		if(state_delay > 0 && (state_phase > 4))
		{
			TC4->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER;		// Will set it to 0 when running
		}
	}
	else if(state_startinfo > 0)
		;
	else if(state_runinfo > 0)
		;
	else
	{
		if(state_syncwait || state_phase >= 4)
		{
			TC4->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER;		// Will set it to 0 when running
			state_phase = 0;
			state_syncwait = 0;
			tc4_prs = 0;
		}
		else if(state_phase >= 1)
		{
			state_correction = true;
		}
		else
		{
			TC4->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER;		// Will set it to 0 when running
			tc4_prs = 0;
		}
	}
	state_counter = 0xFFF;
	tc4_prs = 0xFFF;
	state_attentionprs = 0xFFF;
	state_attentionstate = 0;
}

void State_Init()
{
	gpio_set_pin_direction(LED_R, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_R, true);
	gpio_set_pin_direction(LED_G, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_G, true);
	gpio_set_pin_direction(LED_B, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_B, true);
	
	/*hri_mclk_set_APBBMASK_TCC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	
	hri_mclk_set_APBBMASK_TCC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	
	gpio_set_pin_function(PA08, PINMUX_PA08G_TCC1_WO4);
	gpio_set_pin_function(PA10, PINMUX_PA10G_TCC1_WO6);
	gpio_set_pin_function(PA11, PINMUX_PA11G_TCC1_WO7);
	
	gpio_set_pin_function(PB12, PINMUX_PB12G_TCC0_WO0);
	gpio_set_pin_function(PB13, PINMUX_PB13G_TCC0_WO1);
	gpio_set_pin_function(PB15, PINMUX_PB15G_TCC0_WO3);
	
	pwm_init(&PWM_0, TCC0, _tcc_get_pwm());
	pwm_init(&PWM_1, TCC1, _tcc_get_pwm());
	
	pwm_set_parameters(&PWM_0, PWM_FREQ, 0);
	pwm_enable(&PWM_0);
	
	pwm_set_parameters(&PWM_1, PWM_FREQ, 0);
	pwm_enable(&PWM_1);*/
	
	hri_mclk_set_APBCMASK_TC4_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC4_GCLK_ID, CONF_GCLK_TC4_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while ((TC4->COUNT16.SYNCBUSY.reg) & TC_SYNCBUSY_SWRST) {
	};

	TC4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_MODE_COUNT16;		// 96/4=6MHz
	TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
	TC4->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;									// Match frequency
	TC4->COUNT16.CC[0].reg = ((CONF_GCLK_TC4_FREQUENCY / 16) / 500);				// Max 16-bit (for 32-BIT second timer is also used!!)
	TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;										// Use 16-bit, CC = period
	
	NVIC_DisableIRQ(TC4_IRQn);
	NVIC_ClearPendingIRQ(TC4_IRQn);
	NVIC_SetPriority(TC4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TC4_IRQn);
	
	state_init_led = 3;
}

void State_ComToggle()
{
	if(state_free_toggle)// && state_com_show)
	{
		state_toggle != 1 ? State_SetLed(COLOR_BLUE, 0) : State_SetLed(COLOR_OFF, 0);
		state_toggle = state_toggle != 1 ? 1 : 0;
		state_toggle_timeout = STATE_TIMER_FREQ;			// One sec TO
	}
}

void State_ComToggleRdm()
{
	if(state_free_toggle)// && state_com_show)
	{
		state_toggle != 2 ? State_SetLed(COLOR_AQUA, 0) : State_SetLed(COLOR_OFF, 0);
		state_toggle = state_toggle != 2 ? 2 : 0;
		state_toggle_timeout = STATE_TIMER_FREQ;			// One sec TO
	}
}

void State_ComToggleNoData()
{
	if(state_free_toggle && state_com_show)
	{
		state_toggle ? State_SetLed(COLOR_YELLOW, 0) : State_SetLed(COLOR_OFF, 0);
		state_toggle = !state_toggle;
		state_toggle_timeout = STATE_TIMER_FREQ;			// One sec TO
	}
}

void State_ResetToggleData_Show()
{
	state_com_show = 5 * 60;			// Show for 1 minute
}


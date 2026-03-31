/*
 * state.h
 *
 *  Created on: 13 feb. 2021
 *      Author: Milo
 */

#ifndef STATE_H_
#define STATE_H_

#define STATE_ERROR_MAX				32
#define STATE_ERROR_DEFINED			9

#define STATE_ERROR_NO_SERIAL		0			// No production data
#define STATE_ERROR_NO_EMITTER		1			// No emitter (table)
#define STATE_ERROR_HW_ERROR		2			// Hardware error (DCBM1 error)
#define STATE_ERROR_NO_COM			3			// No communication (but is configured)
#define STATE_ERROR_VIN_LOW			4			// Vin too low
#define STATE_ERROR_OUTPUT_SHORT	5			// Output short circuit
#define STATE_ERROR_NTC				6			// NTC error (internal Tint1 or Tint2)
#define STATE_ERROR_EMITTER_DATA	7			// Emitter data corrupt
#define STATE_ERROR_EMITTER_TYPE	8			// Emitter type not known
#define STATE_ERROR_INCOMPLETE_COM	9			// Incomplete communication (but is configured, but no tokens received)
// Increase STATE_ERROR_DEFINED when error added

#define STATE_WARNING_MAX			32
#define STATE_WARNING_DEFINED		11

#define STATE_WARNING_REBOOT_NEEDED	0		// Reboot needed after production or emitter write
#define STATE_WARNING_VIN_LOW		1		// Vin is too low for optimal operation
#define STATE_WARNING_TEMP_HIGH		2		// Intern temperature high, reducing power
#define STATE_WARNING_NTC1_HIGH		3		// NTC1 temp high, reducing power
#define STATE_WARNING_EMITTER_COM	4		// Emitter COM reset
#define STATE_WARNING_BAD_COMQUAL	5		// Bad Com quality (less than 35% response)
#define STATE_WARNING_SUPPLY_HIGH	6		// Supply voltage high
#define STATE_WARNING_POWER1_HIGH	7		// Power consumption over 36W
#define STATE_WARNING_POWER2_HIGH	8		// Power consumption over 40W on channel 2
#define STATE_WARNING_TIMING_WARN	9		// Timing in task not stable
#define STATE_WARNING_DMX_WARN		10		// Light data frame does not contain enough data
#define STATE_WARNING_UART_OVW_WARN	11		// Uart overflow error
// Increase STATE_WARNING_DEFINED when error added

#define STATE_STARTINFO_MAX		32
#define STATE_STARTINFO_DEFINED	3

#define STATE_START_CANBUS		0
#define STATE_START_IZILINK		1
#define STATE_START_MAIN		2

#define STATE_RUNINFO_MAX		32
#define STATE_RUN_DISCOVER		0
#define STATE_RUN_CONFIG_SET	1

#define STATE_START_COMMISSIONING		0

#ifdef trace_level_t
extern trace_level_t state_trace_lvl;
#endif

#define STATE_PREFIX	"[sta]\t"

typedef enum state_led_e { led_toggle = 0, led_on, led_off } state_led_t;

#ifndef STATE_TRACE_BUILD_LVL
#define STATE_TRACE_BUILD_LVL	3
#endif

#if !defined(DEBUG_UART) || DEBUG_UART == 0
#undef STATE_TRACE_BUILD_LVL
#define STATE_TRACE_BUILD_LVL -1
#endif

#if STATE_TRACE_BUILD_LVL >= 0
#define STATE_TRACE_ERROR(...)		{ if(state_trace_lvl >= TRACE_LEVEL_ERROR)		{ esp_printf(Debug_Putc, STATE_PREFIX, __VA_ARGS__); } }
#else
#define STATE_TRACE_ERROR(...)
#endif
#if STATE_TRACE_BUILD_LVL >= 1
#define STATE_TRACE_WARNING(...)	{ if(state_trace_lvl >= TRACE_LEVEL_WARNING)	{ esp_printf(Debug_Putc, STATE_PREFIX, __VA_ARGS__); } }
#else
#define STATE_TRACE_WARNING(...)
#endif
#if STATE_TRACE_BUILD_LVL >= 2
#define STATE_TRACE_INFO(...)		{ if(state_trace_lvl >= TRACE_LEVEL_INFO)		{ esp_printf(Debug_Putc, STATE_PREFIX, __VA_ARGS__); } }
#else
#define STATE_TRACE_INFO(...)
#endif
#if STATE_TRACE_BUILD_LVL >= 3
#define STATE_TRACE_DEBUG(...)		{ if(state_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, STATE_PREFIX, __VA_ARGS__); } }
#else
#define STATE_TRACE_DEBUG(...)
#endif
#if STATE_TRACE_BUILD_LVL >= 4
#define STATE_TRACE_VERBOSE(...)	{ if(state_trace_lvl >= TRACE_LEVEL_DEBUG)		{ esp_printf(Debug_Putc, STATE_PREFIX, __VA_ARGS__); } }
#else
#define STATE_TRACE_VERBOSE(...)
#endif

#define COLOR_RED			0
#define COLOR_GREEN			1
#define COLOR_BLUE			2
#define COLOR_YELLOW		3
#define COLOR_MAGENTA		4
#define COLOR_AQUA			5
#define COLOR_WHITE			6
#define COLOR_ORANGE		7
#define COLOR_PINK			8
#define COLOR_OFF			9
#define COLOR_YELLOW_LOW	10
#define COLOR_BLUE_LOW		11
#define COLOR_GREEN_LOW		12
#define COLOR_RED_LOW		13
#define COLOR_WHITE_LOW		14
#define COLOR_LIGHT_BLUE	15
#define COLOR_AQUA_MEDIUM	16

// Proto
void State_SetError(uint8_t error);
void State_ClearError(uint8_t error);
bool State_IsErrorActive(uint8_t error);
uint32_t State_GetErrors();
int8_t State_GetHighestError();
char* State_GetHighestErrorText();
void State_SetWarning(uint8_t warning);
void State_ClearWarning(uint8_t warning);
bool State_IsWarningActive(uint8_t warning);
uint32_t State_GetWarnings();
int8_t State_GetHighestWarning();
char* State_GetHighestWarningText();
void State_SetStartInfo(uint8_t startinfo);
void State_ClearStartInfo(uint8_t startinfo);
uint32_t State_GetStartInfo();
uint8_t State_GetHighestStartInfo();
char* State_GetHighestStartInfoText();
void State_SetRunInfo(uint8_t startinfo);
void State_ClearRunInfo(uint8_t startinfo);
void State_SetRunInfo(uint8_t runinfo);
void State_ClearStartInfo(uint8_t runinfo);
uint32_t State_GetRunInfo();


void State_ComToggle();
void State_ComToggleNoData();
void State_ComToggleRdm();

void State_SetCan(uint8_t state);

void State_debug(uint8_t *data, uint8_t length);
void State_Init();
void State_TimerSync();

void State_LedDmxIn(state_led_t state);
void State_LedDmxOut(state_led_t state);
void State_LedArtnet(state_led_t state);
void State_LedContact(state_led_t state);
void State_LedDali(state_led_t state);
void State_LedInitStart();
void State_LedInitReady();
bool State_IsLedInitReady();
void State_SetAttentionInfo(uint8_t color_idx1, uint8_t color_idx2, uint16_t time, uint8_t speed);
void State_ClearAttentionInfo();
bool State_IsAttentionBusy();
void State_ResetToggleData_Show();

#endif /* STATE_H_ */

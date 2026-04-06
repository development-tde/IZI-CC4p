/*
 * iziplus_module_def.c
 *
 * Created: 26-2-2022 17:28:14
 *  Author: Milo
 */ 

#include "includes.h"
#include "iziplus_module_def.h"
#include "version.h"
#include "appconfig.h"
#include "iziplus_dataframe.h"
#include "iziplus_driver.h"
#include "stepdown.h"
#include "analog.h"
#include "izi_output.h"
#include "at30tse.h"
#include "crc.h"
#include "11LC160.h"
#include "dmx.h"
#include "math.h"
#include "dmx_rdm_models.h"
#include "dmx_rdm_module.h"

#define TEMP_MAX_RESOLUTION			2000					// Max calculation for resolution of temp and power reduce, checked per 100ms
#define TEMP_LIMIT_START_OFFSET		5						// 5 degrees before limit the power reduce will start
#define MAX_INTERN_TEMP				104						// Max 104 degrees intern (104 = off, from 104 - TEMP_LIMIT_START_OFFSET reduce will start)
#define MAX_NTC_TEMP				90						// Max 90 degrees intern (90 = off, from 90 - TEMP_LIMIT_START_OFFSET reduce will start)
#define POWER_MAX_RESOLUTION		TEMP_MAX_RESOLUTION		// 0.05% per 100ms
#define POWER_MAX_CC4				120000					// in 1mW = 120Watt (while 4x40W)

#define MODE_CTRL_UNKNOWN		0
#define MODE_CTRL_RGBW			1
#define MODE_CTRL_TW			2
#define MODE_CTRL_WD			3
#define MODE_CTRL_DIRECT		4			// No RGBW/Tunable/WD just dim
#define MODE_CTRL_WWCW			5			// Warm white cold white

uint8_t module_channels_amount = 0;			// AMount of channels in current mode
iziplus_cpustate_u module_cpu_state;
uint8_t module_monitor[IZI_MAX_MONITOR];
uint8_t module_config[IZI_MAX_CONFIG];
uint16_t module_identify_time = 0, module_identify_prs = 0, module_identify_mode = 0, module_identify_unlock_time = 0;
uint16_t module_pup_time  = 0;
uint16_t module_reset_report;
extern uint8_t reset_cause;

static bool iziplus_last_comcheck = false;
static uint8_t iziplus_prs_comcheck = 0;
static bool iziplus_data_wasok = false;					// Has light data ever been received?

volatile uint8_t overrule_level_chx[MAX_DIM_CHANNELS];					// Overrule level, overriding channel level
volatile uint16_t overrule_time_chx = 0;								// Time to overrule channel level
volatile uint8_t overrule_prio_chx = 0;
static uint16_t iziplus_level_chx[MAX_DIM_CHANNELS];
uint16_t iziplus_devtype, iziplus_devtype_base;

uint16_t ntc_temp_master = TEMP_MAX_RESOLUTION;				// NTC reduce in promille
uint16_t intern_temp_master = TEMP_MAX_RESOLUTION;			// Intern temp reduce in promille

volatile int16_t max_intern_temp_raw = 0, limit_intern_temp_raw;
volatile int16_t max_ntc_temp_raw = 0, limit_ntc_temp_raw;

uint16_t power_master = TEMP_MAX_RESOLUTION;
uint32_t iziplus_maxpower;

uint8_t timer_prs = 0;

volatile uint16_t safety_time_ch[MAX_DIM_CHANNELS] = { 0, 0, 0, 0 };						// Time to overrule channel level to 0 for safety

//uint8_t monitor_timer_idx_prs = 0;
uint8_t monitor_idx = 0, monitor_idx_ext = 0;

uint16_t monitor_avg_level[MAX_DIM_CHANNELS];

extern volatile stepdown_control_t stepdown_ctrl_chx[MAX_DIM_CHANNELS];

iziplus_fixture_def	iziplus_fixdef;
uint16_t  iziplus_config_size;
uint16_t  iziplus_monitor_size;
bool iziplus_init_rdy = false;

// Proto
void print_resetcause();

void IziPlus_Module_Fixture_CC4();
bool IziPlus_Module_SetFixtureParameters();

extern const uint8_t ntc_table[512];
extern boot_t boot_data;

int16_t Adc_GetTempRaw(uint8_t degrees)
{
	for(uint16_t i = 0; i < 512; i++)
	{
		if(ntc_table[i] == degrees)
		{
			return (int16_t)(i << 3);				// Give back raw value of AD (convert 8 to 12 bit)
		}
	}
	return -1;
}

uint16_t Adc_GetTintRaw()
{
	return 3000;
}

/************************************************************************/
/* Code                                                                 */
/************************************************************************/
bool IziPlus_GetDevType()
{
	iziplus_devtype_base = IZIPLUS_DEVTYPE_BASE;
	iziplus_devtype = iziplus_devtype_base;
	
	return true;
}

// Init current parameters
bool IziPlus_Module_Init()
{
	bool res = true;
	//print_resetcause();
	
	module_reset_report = 30;		// Report reset cause for 3 seconds
	module_cpu_state.w = 0;
	if(reset_cause == RESET_CAUSE_POWER_UP)
		module_cpu_state.u.powerup = true;
	else if(reset_cause == RESET_CAUSE_EXTRST)
		module_cpu_state.u.extreset = true;
	else if(reset_cause == RESET_CAUSE_BOD)
		module_cpu_state.u.brownout = true;
	else if(reset_cause == RESET_CAUSE_SYSRST)
		module_cpu_state.u.sw_reset = true;
	else if(reset_cause == RESET_CAUSE_WATCHDOG)
		module_cpu_state.u.watchdog = true;
	
	IziPlus_GetDevType();
	IziPlus_Module_SetFixtureParameters();
	
	AppConfig_CheckConfig();					// Check if all config is within limits
	IziPlus_GetDevType();						// Make sure the right color tables are assigned
	IziPlus_Module_Update();
	
	iziplus_init_rdy = true;
	
#ifndef MS_TEST_NO_EMITTER
	memset(monitor_avg_level, 0, sizeof(monitor_avg_level));
#endif
	
	return res;
}

bool IziPlus_Module_InitReady()
{
	return iziplus_init_rdy;
}

bool IziPlus_Module_SetFixtureParameters()
{
	IziPlus_Module_Fixture_CC4();					// Load known settings
	
	iziplus_config_size = 0;
	for(int i = 0; i < iziplus_fixdef.max_configs; i++)
		iziplus_config_size += iziplus_fixdef.configs[i].size;
	
	iziplus_monitor_size = 0;
	for(int i = 0; i < iziplus_fixdef.max_monitors; i++)
		iziplus_monitor_size += iziplus_fixdef.monitors[i].size;
		
	return true;
}


void IziPlus_Module_Fixture_CC4()
{
	strcpy(iziplus_fixdef.model_name, "IZI-DriveCC4+");
		
	iziplus_fixdef.max_modes = 9;
	static const iziplus_mode_def modes[9] =
	{
		{ .channels = 1, .name = "1 Channel"},
		{ .channels = 5, .name = "4 Chan Master"},
		{ .channels = 4, .name = "4 Channel"},
		{ .channels = 1, .name = "1 Output"},
		{ .channels = 2, .name = "TunableWhite"},
		{ .channels = 1, .name = "WarmDimming"},
		{ .channels = 2, .name = "Changeover TW"},
		{ .channels = 1, .name = "Changeover WD"},
		{ .channels = 8, .name = "4 Channel 16-bit"}
	};
	memcpy(iziplus_fixdef.modes, modes, sizeof(modes));
	iziplus_fixdef.default_mode = 2;
	
	iziplus_fixdef.max_configs = 7;
	static const iziplus_config_def configs[7] =
	{
		{ .size = 1, .idx = 0, .ref = 1, .name = "Current set", .min = 0, .max = 18, .dflt = 0 },
		{ .size = 1, .idx = 1, .ref = 34, .name = "Filter mode", .min = 0, .max = 3, .dflt = 0},
		{ .size = 1, .idx = 2, .ref = 12, .name = "NTC1 Temp Prot", .min = 60, .max = 90, .dflt = 70 },
		{ .size = 1, .idx = 3, .ref = 13, .name = "NTC2 Temp Prot", .min = 60, .max = 90, .dflt = 70 },
		{ .size = 1, .idx = 4, .ref = 16, .name = "Channel Swap", .min = 0, .max = 23, .dflt = 0 },
		{ .size = 1, .idx = 5, .ref = 33, .name = "Contact 1 Map", .min = 0, .max = 32, .dflt = 0 },
		{ .size = 1, .idx = 6, .ref = 35, .name = "Dim curve", .min = 0, .max = 3, .dflt = 2 },
	};		// check CONFIG_SIZE if chang.name = "Frequency mode", .min = 0, .max = (CONFIG_MAX_FREQ - 1), .dflt = 0 }ed
	memcpy(iziplus_fixdef.configs, configs, sizeof(configs));
	
	/*iziplus_config_size = 0;
	for(int i = 0; i < iziplus_fixdef.max_configs; i++)
		iziplus_config_size += iziplus_fixdef.configs[i].size;
	*/
	iziplus_fixdef.max_monitors = 19;
	static const iziplus_monitor_def monitors[19] =
	{
		{ .size = 1, .idx = 0,  .ref = 1,  /*.name = "Supply Voltage" */ },
		{ .size = 1, .idx = 1,  .ref = 26, /*.name = "Power" */ },
		{ .size = 1, .idx = 3,  .ref = 23, /*.name = "Output" */ },
		{ .size = 1, .idx = 4,  .ref = 4,  /*.name = "Internal Temp" */ },
		{ .size = 1, .idx = 5,  .ref = 5,  /*.name = "NTC1 Temp" */ },
		{ .size = 1, .idx = 6,  .ref = 6,  /*.name = "NTC2 Temp" */ },
		{ .size = 1, .idx = 7,  .ref = 13, /*.name = "Max IntTemp" */ },
		{ .size = 1, .idx = 8,  .ref = 14, /*.name = "Max NTC1 Temp" */ },
		{ .size = 1, .idx = 9,  .ref = 15, /*.name = "Max NTC2 Temp" */ },
		{ .size = 1, .idx = 10, .ref = 19, /*.name = "Com Quality" */ },
		{ .size = 1, .idx = 11, .ref = 25, /*.name = "Active Contacts" */ },
		{ .size = 1, .idx = 12, .ref = 21, /*.name = "Status" */ },
		{ .size = 2, .idx = 13, .ref = 30, /*.name = "Forward voltage x" */ },
		{ .size = 2, .idx = 15, .ref = 32, /*.name = "Uptime" */ },
		{ .size = 2, .idx = 17, .ref = 33, /*.name = "Operating Time x" */ },
		{ .size = 2, .idx = 19, .ref = 36, /*.name = "Frequency x" */ },
		{ .size = 2, .idx = 21, .ref = 34, /*.name = "Min output" */ },
		{ .size = 1, .idx = 22, .ref = 31, /*.name = "Min supply voltage" */ },
		{ .size = 2, .idx = 23, .ref = 43, /*.name = "Debug" */ },
	};		// MONITOR_SIZE
	memcpy(iziplus_fixdef.monitors, monitors, sizeof(monitors));
	/*iziplus_monitor_size = 0;
	for(int i = 0; i < iziplus_fixdef.max_monitors; i++)
		iziplus_monitor_size += iziplus_fixdef.monitors[i].size;	*/
}

/************************************************************************/
/* Call after every config write and after init                         */
/************************************************************************/
void IziPlus_Module_Update()
{
	if(appConfig->mode < MODE_AMOUNT)
		module_channels_amount = iziplus_fixdef.modes[appConfig->mode].channels;
	
	max_intern_temp_raw = Adc_GetTempRaw(MAX_INTERN_TEMP);
	limit_intern_temp_raw = Adc_GetTempRaw(MAX_INTERN_TEMP - TEMP_LIMIT_START_OFFSET);
	max_ntc_temp_raw = Adc_GetTempRaw(MAX_NTC_TEMP);
	limit_ntc_temp_raw = Adc_GetTempRaw(MAX_NTC_TEMP - TEMP_LIMIT_START_OFFSET);
}

bool IziPlus_Module_TemperatureCheck(int16_t temp, uint16_t *temp_master, int16_t max_config, int16_t limit_config)
{
	if(temp <= limit_config)			// Temp above limit, reduce level (looks weird because value gets lower on a higher temperature, because the raw ad value is used)
	{
		if(temp > max_config)			// Still under max temp (where it should be 0)
		{
			uint16_t level_limit = (temp - max_config) * (TEMP_MAX_RESOLUTION / (limit_config - max_config));
			if(*temp_master > level_limit)
			{
				if(((*temp_master - level_limit) >= 128) && (*temp_master >= 4))
					*temp_master -= 4;		// -1 is 0.20% per 100ms
				else if(*temp_master > 0)
					*temp_master -= 1;		// -1 is 0.05% per 100ms
			}
			else if((*temp_master < level_limit) && (*temp_master < TEMP_MAX_RESOLUTION))
				*temp_master += 1;
		}
		else if(*temp_master > 10)		// Faster (max 10s) to 0% when on or over limit
			*temp_master -= 10;
		else
			*temp_master = 0;
		
		return true;					// Report temp too high, still reducing
	}
	else if(*temp_master < (TEMP_MAX_RESOLUTION - 10))
	{
		*temp_master += 10;
		return true;					// Report temp too high, still reducing
	}
	else
		*temp_master = TEMP_MAX_RESOLUTION;
	
	return false;					// Report temp too high, still reducing
}

bool IziPlus_Module_TemperatureCelciusCheck(int16_t temp, uint16_t *temp_master, int16_t max_config, int16_t limit_config)
{
	if(temp >= limit_config)			// Temp above limit, reduce level (looks weird because value gets lower on a higher temperature, because the raw ad value is used)
	{
		if(temp < max_config)			// Still under max temp (where it should be 0)
		{
			uint16_t level_limit = (max_config - temp) * (TEMP_MAX_RESOLUTION / (max_config - limit_config));
			if(*temp_master > level_limit)
			{
				if(((*temp_master - level_limit) >= 128) && (*temp_master >= 4))
					*temp_master -= 4;		// -1 is 0.20% per 100ms
				else if(*temp_master > 0)
					*temp_master -= 1;		// -1 is 0.05% per 100ms
			}
			else if((*temp_master < level_limit) && (*temp_master < TEMP_MAX_RESOLUTION))
				*temp_master += 1;
		}
		else if(*temp_master > 10)		// Faster (max 10s) to 0% when on or over limit
			*temp_master -= 10;
		else
			*temp_master = 0;
		
		return true;					// Report temp too high, still reducing
	}
	else if(*temp_master < (TEMP_MAX_RESOLUTION - 10))
	{
		*temp_master += 10;
		return true;					// Report temp too high, still reducing
	}
	else
		*temp_master = TEMP_MAX_RESOLUTION;
	
	return false;					// Report temp too high, still reducing
}

uint32_t IziPlus_Module_TempLimit(uint32_t level)
{
	uint16_t temp_master = intern_temp_master > ntc_temp_master ? ntc_temp_master : intern_temp_master;		// Take the lowest level
	if(power_master < temp_master)
		temp_master = power_master;
	
	if(temp_master < TEMP_MAX_RESOLUTION)
		level = (uint64_t)((uint64_t)level * (uint64_t)temp_master)/TEMP_MAX_RESOLUTION;
	
	return level;
}

//static uint16_t reducing = 0;
//static uint32_t last_power = 0;
bool IziPlus_Module_PowerCheck()
{
	uint16_t last_power_master = power_master;
	uint32_t power = StepDown_GetPower();
	uint16_t delta = 0;
	
	iziplus_maxpower = POWER_MAX_CC4;
	if(power > iziplus_maxpower)
	{
		/*if(last_power < power)
		{
			if(reducing < 4)
				reducing++;
		}*/
			
		delta = (power - iziplus_maxpower) / 64; // (64 >> (reducing / 2));
		if(delta == 0)
			delta = 1;
		else if(delta > 256)
			delta = 256;
			
		if(power_master < delta)
			power_master = 0;
		else
			power_master -= delta;
		
		//power_master = (POWER_MAX_RESOLUTION * iziplus_maxpower) / power;
	}
	else if(power_master < POWER_MAX_RESOLUTION)
	{
		//reducing = 0;
		delta = (iziplus_maxpower - power) / 64; 
		if(delta == 0)
			delta = 1;
		else if(delta > 256)
			delta = 256;
		
		if(power_master + delta > POWER_MAX_RESOLUTION)
			power_master = POWER_MAX_RESOLUTION;
		else
			power_master += delta;
	}
	else
	{
		//reducing = 0;
		power_master = POWER_MAX_RESOLUTION;
	}
	//stepdown_ctrl_chx[0].test = power;
	//stepdown_ctrl_chx[0].test2 = delta;
	
	power_master = Filter_LowPass(last_power_master, power_master, 3);
	//last_power = power;
	
	return power_master != POWER_MAX_RESOLUTION;					// Report power too high, still reducing = true
}

/*
 * 100 ms timer timed by OS. Do not call OS routines from here
 */
void IziPlus_Module_Timer100ms()
{
	if(++module_identify_prs >= 50)
		module_identify_prs = 0;
	if(overrule_time_chx > 0)
		overrule_time_chx--;
		
	for(int i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		if(safety_time_ch[i] > 0)
			safety_time_ch[i]--;
	}
	
	if(module_reset_report > 0)
	{
		if(--module_reset_report == 0)
		{
			module_cpu_state.u.powerup = false;		// Clear reset cause
			module_cpu_state.u.extreset = false;
			module_cpu_state.u.brownout = false;
			module_cpu_state.u.sw_reset = false;
			module_cpu_state.u.watchdog = false;
			IziPlus_Module_Update();
		}
	}
	
	if(Analog_Ready())
	{
		if(IziPlus_Module_TemperatureCheck(Adc_GetNtcRaw(), &ntc_temp_master,  max_ntc_temp_raw,  limit_ntc_temp_raw))
			State_SetWarning(STATE_WARNING_NTC1_HIGH);
		else
			State_ClearWarning(STATE_WARNING_NTC1_HIGH);
	
		if(IziPlus_Module_TemperatureCheck(Adc_GetTemperatureRaw(), &intern_temp_master, max_intern_temp_raw, limit_intern_temp_raw))
			State_SetWarning(STATE_WARNING_TEMP_HIGH);
		else
			State_ClearWarning(STATE_WARNING_TEMP_HIGH);
	}
	
	if(module_identify_time > 0)
	{
		if(--module_identify_time > 0)
		{
			if(module_identify_mode == IZIPLUS_IDENTMODE_BLINK)
			{
				if(module_identify_prs <= 1)
					IziPlus_Module_Overrule_Ch1234(0xFFFFFFFF, 400, PRIO_OVERRULE_HIGH);
				else if(module_identify_prs == 4)
					IziPlus_Module_Overrule_Ch1234(0, 200, PRIO_OVERRULE_HIGH);
					
				if(module_identify_prs == 6)
					IziPlus_Module_Overrule_Chx(0, 0xFF, 200, PRIO_OVERRULE_HIGH);
				else if(module_identify_prs == 8)
					IziPlus_Module_Overrule_Chx(0, 0, 200, PRIO_OVERRULE_HIGH);
			
				if(module_identify_prs == 10 || module_identify_prs == 14)
					IziPlus_Module_Overrule_Chx(1, 0xFF, 200, PRIO_OVERRULE_HIGH);
				else if(module_identify_prs == 12 || module_identify_prs == 16)
					IziPlus_Module_Overrule_Chx(1, 0, 200, PRIO_OVERRULE_HIGH);
					
				if(module_identify_prs == 18 || module_identify_prs == 22 || module_identify_prs == 26)
					IziPlus_Module_Overrule_Chx(2, 0xFF, 200, PRIO_OVERRULE_HIGH);
				else if(module_identify_prs == 20 || module_identify_prs == 24 || module_identify_prs == 28)
					IziPlus_Module_Overrule_Chx(2, 0, 200, PRIO_OVERRULE_HIGH);
					
				if(module_identify_prs == 30 || module_identify_prs == 34 || module_identify_prs == 38 || module_identify_prs == 42)
					IziPlus_Module_Overrule_Chx(3, 0xFF, 200, PRIO_OVERRULE_HIGH);
				else if(module_identify_prs == 32 || module_identify_prs == 36 || module_identify_prs == 40 || module_identify_prs == 44)
					IziPlus_Module_Overrule_Chx(3, 0, module_identify_prs == 44 ? 600 : 200, PRIO_OVERRULE_HIGH);
			}
			else if(module_identify_mode == IZIPLUS_IDENTMODE_SPOTON)
			{
				IziPlus_Module_Overrule_Ch1234(0xFFFFFFFF, 1000, PRIO_OVERRULE_HIGH);
			}
			else if(module_identify_mode == IZIPLUS_IDENTMODE_QUALITY)
			{
				for(int i = 0; i < MAX_DIM_CHANNELS; i++)
				{
					uint8_t level = (iziplus_data_comquality() * 0xFF) / 100;
					IziPlus_Module_Overrule_Chx(i, level, 1000, PRIO_OVERRULE_HIGH);
				}
			}
			else if(module_identify_unlock_time)
			{
				if(module_identify_mode == IZIPLUS_IDENTMODE_FACTORY_DFLT)
				{
					State_SetAttentionInfo(COLOR_RED, COLOR_WHITE, 4, 2);
					AppConfig_Default();
					AppLog_Default(false);
				}
				else if(module_identify_mode == IZIPLUS_IDENTMODE_MAX_DFLT)
				{
					State_SetAttentionInfo(COLOR_RED, COLOR_PINK, 4, 2);
					AppLog_Default(false);
				}
				else if(module_identify_mode == IZIPLUS_IDENTMODE_MAXALL_DFLT)
				{
					State_SetAttentionInfo(COLOR_RED, COLOR_BLUE, 4, 2);
					AppLog_Default(true);
				}
				else if(module_identify_mode == IZIPLUS_IDENTMODE_RESET)
				{
					StepDown_Close(1000);
					vTaskDelay(200);
					AppLog_Set();
					
					boot_data.action = BOOT_ACTION_STARTAPP;					// Reboot
					boot_data.magic = BOOT_MAGIC_VALUE;
					NVIC_SystemReset();
				}
			}
		}
		else
		{
			IziPlus_Module_Overrule_Ch1234(0, 0, PRIO_OVERRULE_HIGH);
			module_cpu_state.u.identify = 0;
		}
	}
	if(++timer_prs >= 10)
	{
		appLogData->operating_sec++;
		for(int i = 0; i < MAX_DIM_CHANNELS; i++)
		{
			if(iziplus_level_chx[i])
			{
				appLogData->active_sec++;
				break;
			}
		}
		for(int i = 0; i < MAX_DIM_CHANNELS; i++)
		{
			if(iziplus_level_chx[i])
				appLogData->active_ch_sec[i]++;
		}
		
		if(module_pup_time < (60 * 60))				// Keep the timer after power-up, clip at one hour
			module_pup_time++;
		timer_prs = 0;
		
		/*if(++monitor_timer_idx_prs >= 3)			// Change every 3 seconds
		{
			monitor_timer_idx_prs = 0;
			if(++monitor_idx >= 4)
				monitor_idx = 0;
		}*/
	}
}

bool IziPlus_Module_IsSafetyOff_Ch1()
{
	return (safety_time_ch[0] > 0);
}

bool IziPlus_Module_IsSafetyOff_Ch2()
{
	return (safety_time_ch[1] > 0);
}

bool IziPlus_Module_IsSafetyOff_Ch3()
{
	return (safety_time_ch[2] > 0);
}

bool IziPlus_Module_IsSafetyOff_Ch4()
{
	return (safety_time_ch[3] > 0);
}

bool IziPlus_Module_IsSafetyOff_Chx(uint8_t idx)
{
	if(idx >= MAX_DIM_CHANNELS)
		return false;
	
	return safety_time_ch[idx] > 0;
}

void IziPlus_Module_SafetyOff_Chx(uint8_t idx, uint16_t time)
{
	if(idx >= MAX_DIM_CHANNELS)
		return;
		
	safety_time_ch[idx] = time / 100;
}


void IziPlus_Module_IdentifySync()
{
	module_identify_prs = 0;
}

bool IziPlus_Module_IsIdentifying()
{
	return (module_identify_time > 0);
}

iziplus_cpustate_u IziPlus_Module_GetCpuState()
{
	return module_cpu_state;
}

uint8_t IziPlus_Module_MonitorValue(uint8_t ref, uint8_t *bfr)
{
	if(ref == 1)					// Supply Voltage in V
	{
		*bfr = (uint8_t)((Analog_GetSupplyVoltage() + 125)/250);
		return 1;		
	}
	else if(ref == 19)				// Com Quality (0 to 100%)
	{
		*bfr = iziplus_data_comquality();
		return 1;
	}
	else if(ref == 26)				// Power per 0.01V (div 100)
	{
		uint32_t power = StepDown_GetPower() / 10;
		bfr[0] = (uint8_t)(power >> 0);
		bfr[1] = (uint8_t)(power >> 8);
		stepdown_ctrl_chx[0].test = power;
		return 2;
	}
	else if(ref == 23)				// Output (100% is no limiter active)
	{
		uint8_t temp_limit = intern_temp_master > ntc_temp_master ? (ntc_temp_master/(TEMP_MAX_RESOLUTION/100)) : (intern_temp_master/(TEMP_MAX_RESOLUTION/100));
		uint8_t power_limit = State_IsWarningActive(STATE_WARNING_POWER_TOT_HIGH) ? (power_master/(TEMP_MAX_RESOLUTION/100)) : 100;
		*bfr = power_limit < temp_limit ? power_limit : temp_limit;
		if(module_monitor[3] < appLogData->min_output)
			appLogData->min_output = module_monitor[3];
		return 1;
	}
	else if(ref == 4)				// Internal Temp
	{
		*bfr = Adc_GetTemperature();
		return 1;
	}
	else if(ref == 5)				// NTC1
	{
		*bfr = Adc_GetNtc();
		return 1;
	}
	else if(ref == 6)				// NTC2
	{
		*bfr = Adc_GetNtc2();
		return 1;
	}
	else if(ref == 13)				// Max Intern Temp
	{
		*bfr = Adc_GetTemperatureMax();
		return 1;
	}
	else if(ref == 14)				// Max NTC1 Temp
	{
		*bfr = Adc_GetNtcMax();
		return 1;
	}
	else if(ref == 15)				// Max NTC2 Temp
	{
		*bfr = Adc_GetNtc2Max();
		return 1;
	}
	else if(ref == 25)
	{	
		*bfr = IziInput_GetExtInput();
		return 1;
	}
	else if(ref == 32)				// Up Time (time the device is powered and running) in seconds
	{
		uint16_t operating_days = appLogData->operating_sec / (24*60*60/10);		// Per 10th of day
		bfr[0] = (uint8_t)(operating_days >> 0);
		bfr[1] = (uint8_t)(operating_days >> 8);
		return 2;
	}
	else if(ref == 35)				// Up Time (time at least one channel is > 0%) in seconds
	{
		uint16_t active_days = appLogData->active_sec / (24*60*60/10);				// Per 10th of day
		bfr[0] = (uint8_t)(active_days >> 0);
		bfr[1] = (uint8_t)(active_days >> 8);
		return 2;
	}
	else if(ref == 21)				// Status index
	{
		int8_t error = State_GetHighestError();
		uint8_t state = 0;
		if(error >= 0)
		{
			if(error == STATE_ERROR_VIN_LOW)
				state = 132;
			if(error == STATE_ERROR_OUTPUT1_SHORT)
				state = 138;
			else if(error == STATE_ERROR_OUTPUT2_SHORT)
				state = 140;
			else if(error == STATE_ERROR_OUTPUT3_SHORT)
				state = 142;
			else if(error == STATE_ERROR_OUTPUT4_SHORT)
				state = 144;
			else if(error == STATE_ERROR_VIN_LOW)
				state = 132;
			else if(error == STATE_ERROR_HW_ERROR)
				state = 159;
		}
		int8_t warning = State_GetHighestWarning();
		if(warning >= 0 && state == 0)
		{
			if(warning == STATE_WARNING_OUTPUT1_OPEN)
				state = 137;
			else if(warning == STATE_WARNING_OUTPUT2_OPEN)
				state = 139;
			else if(warning == STATE_WARNING_OUTPUT3_OPEN)
				state = 141;
			else if(warning == STATE_WARNING_OUTPUT4_OPEN)
				state = 143;
			else if(warning == STATE_WARNING_TEMP_HIGH)
				state = 128;
			else if(warning == STATE_WARNING_NTC1_HIGH)
				state = 130;
			else if(warning == STATE_WARNING_NTC2_HIGH)
				state = 131;
			else if(warning == STATE_WARNING_SUPPLY_HIGH)
				state = 133;
			else if(warning == STATE_WARNING_BAD_COMQUAL)
				state = 148;
			else if(warning == STATE_WARNING_POWER_TOT_HIGH || warning == STATE_WARNING_POWER1_HIGH || warning == STATE_WARNING_POWER2_HIGH || warning == STATE_WARNING_POWER3_HIGH || warning == STATE_WARNING_POWER4_HIGH)
				state = 160;
			else if(warning == STATE_WARNING_UART_OVW_WARN)
				state = 151;
			else if(warning == STATE_WARNING_VIN_LOW)
				state = 132;
			else if(warning == STATE_WARNING_REBOOT_NEEDED)
				state = 157;
		}
		*bfr = state;
		return 1;
	}
	else if(ref == 30)				// Forward voltage (all channels)
	{
		uint16_t vled = 0;
		/*if(monitor_idx == 0)			// Check current channel (order of WRGB)
		{
			vled = Adc_GetVled1();
		}
		else if(monitor_idx == 1)
		{
			vled = Adc_GetVled2();
		}
		else if(monitor_idx == 2)
		{
			vled = Adc_GetVled3();
		}
		else
		{
			vled = Adc_GetVled4();
		}*/
		vled = ADC_RAW_MV(stepdown_ctrl_chx[monitor_idx].vled_ad);
		
		if(vled < 1000)
			vled = 0;
		vled = (vled / 4) + (monitor_idx << 14);
		bfr[0] = (uint8_t)(vled >> 0);
		bfr[1] = (uint8_t)(vled >> 8);
		return 2;
	}
	else if(ref == 34)				// How much was it limited ever? (100% is never)
	{
		*bfr = appLogData->min_output;
		return 1;
	}
	else if(ref == 31)				// Lowest supply voltage (ever) in V
	{
		*bfr = (uint8_t)((appLogData->supply_min + 125)/250);
		return 1;
	}
	else if(ref == 33)					// Active time per channel, per quarter of a day
	{
		uint8_t idx_offset = (monitor_idx << 6);
		uint16_t active_days_chx = appLogData->active_ch_sec[monitor_idx] / (24*60*60/4);
		bfr[0] = (uint8_t)(active_days_chx >> 0);
		bfr[1] = (uint8_t)(active_days_chx >> 8) + idx_offset;
		//OS_TRACE_INFO("Send active days[%d]: 0x%x%02x (%d)\r\n", monitor_idx, bfr[1], bfr[0], emitterLogData->active_ch_sec[monitor_idx]);
		return 2;
	}
	else if(ref == 36)					// Frequency per channel
	{
		uint8_t idx_offset = (monitor_idx << 6);
		uint16_t avg_level = stepdown_ctrl_chx[monitor_idx].level_curve > 0 ? stepdown_ctrl_chx[monitor_idx].freq_new / 10 : 0;		// Avg with two decimals
		bfr[0] = (uint8_t)(avg_level >> 0);
		bfr[1] = (uint8_t)(avg_level >> 8) + idx_offset;
		//OS_TRACE_INFO("Send avg level[%d]: 0x%x%02x (%d %d %d)\r\n", monitor_idx, bfr[1], bfr[0], avg_level, (emitterLogData->level_sum_ch[monitor_idx] * 10), (emitterLogData->active_ch_sec[monitor_idx] / 60));
		return 2;
	}
	else if(ref == 43)			// Trace/Debug 16-bit value
	{
		uint16_t dbg = stepdown_ctrl_chx[0].test;
		bfr[0] = (uint8_t)(dbg >> 0);
		bfr[1] = (uint8_t)(dbg >> 8);
		return 2;
	}	
	
	return 0;
}

uint8_t *IziPlus_Module_GetMonitor()
{
	if(++monitor_idx >= MAX_DIM_CHANNELS)
		monitor_idx = 0;
	if(++monitor_idx_ext >= MAX_DIM_CHANNELS * 4)
		monitor_idx_ext = 0;
		
	for(int i = 0; i < iziplus_fixdef.max_monitors; i++)
	{
		IziPlus_Module_MonitorValue(iziplus_fixdef.monitors[i].ref, module_monitor + iziplus_fixdef.monitors[i].idx);
	}
	return module_monitor;
}

uint8_t IziPlus_Module_SetConfigValue(uint8_t ref, uint8_t *bfr, uint8_t len)
{
	if(len == 0)
		return 0;
	
	appconfig_t *appconfig = AppConfig_GetPtr();
	if(ref == 1)						// Current in mA
	{
		appconfig->config[APPCFG_CURRENT] = bfr[0];
		return 1;
	}
	else if(ref == 12)					// NTC1 temp protect
	{
		appconfig->config[APPCFG_NTC1] = bfr[0];
		return 1;
	}
	else if(ref == 13)					// NTC2 temp protect
	{
		appconfig->config[APPCFG_NTC2] = bfr[0];
		return 1;
	}
	else if(ref == 34)					// Filter mode
	{
		appconfig->config[APPCFG_FILTERMODE] = bfr[0];
		return 1;
	}
	else if(ref == 35)					// Dim curve
	{
		appconfig->config[APPCFG_CURVE] = bfr[0];
		return 1;
	}
	else if(ref == 16)					// Swap
	{
		appconfig->config[APPCFG_CHAN_SWAP] = bfr[0];
		return 1;
	}
	else if(ref == 33)					// Contact mapping
	{
		appconfig->config[APPCFG_CONTACT_MAP] = bfr[0];
		return 1;
	}
	else if(ref == CFGTYPE_REFID_DMXFAIL_PUP)
	{
		appconfig->dmxfail = (appconfig->dmxfail & 0xF0) | (bfr[0] & 0x0F);
	}
	else if(ref == CFGTYPE_REFID_DMXFAIL_OPERATION)
	{
		appconfig->dmxfail = (appconfig->dmxfail & 0x0F) | ((bfr[0] & 0x0F) << 4);
	}
	return 0;
}

/************************************************************************/
/* Write one or multiple parameters received to memory, can be multiple 
  parameters if in the right order. Write is prepared, AppConfig_Set 
  has to be called elsewhere. Returns the amount of parameters written. */
/************************************************************************/
uint8_t IziPlus_Module_SetConfigByIndex(uint8_t idx, uint8_t *bfr, uint8_t len)
{
	uint8_t l = len, amount = 0;
	for(int i = 0; i < iziplus_fixdef.max_configs; i++)
	{
		if(iziplus_fixdef.configs[i].idx == idx)
		{
			uint8_t size = iziplus_fixdef.configs[i].size;
			if(l >= size)
			{
				IziPlus_Module_SetConfigValue(iziplus_fixdef.configs[i].ref, bfr + idx, size);
				l -= size;
				amount++;
				idx += size;
			}
			else
				break;	
		}
	}
	return amount;
}

/************************************************************************/
/* Get config value by given reference                                  */
/************************************************************************/
uint8_t IziPlus_Module_ConfigValue(uint8_t ref, uint8_t *bfr)
{
	if(ref == 1)						// Current in mA
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_CURRENT]);
		return 1;
	}
	else if(ref == 12)					// NTC1 Temp protect
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_NTC1]);
		return 1;
	}
	else if(ref == 13)					// NTC2 Temp protect
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_NTC2]);
		return 1;
	}
	else if(ref == 34)					// Filter mode
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_FILTERMODE]);
		return 1;
	}
	else if(ref == 35)					// Dim curve
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_CURVE]);
		return 1;
	}
	else if(ref == 16)					// Channel swap
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_CHAN_SWAP]);
		return 1;
	}
	else if(ref == 33)					// Contact mapping
	{
		*bfr = (uint8_t)(appConfig->config[APPCFG_CONTACT_MAP]);
		return 1;
	}
	else if(ref == CFGTYPE_REFID_DMXFAIL_PUP)
	{
		*bfr = (uint8_t)(appConfig->dmxfail & 0x0F);
		return 1;
	}
	else if(ref == CFGTYPE_REFID_DMXFAIL_OPERATION)
	{
		*bfr = (uint8_t)((appConfig->dmxfail & 0xF0) >> 4);
		return 1;
	}
	return 0;
}

uint8_t *IziPlus_Module_GetConfig()
{
	for(int i = 0; i < iziplus_fixdef.max_configs; i++)
	{
		IziPlus_Module_ConfigValue(iziplus_fixdef.configs[i].ref, module_config + iziplus_fixdef.configs[i].idx);
	}
	return module_config;
}

void IziPlus_Module_SetIdentify(uint8_t mode, uint8_t time)
{
	// Todo: Handle all modes
	if(time > 0)
	{
		State_SetAttentionInfo(COLOR_MAGENTA, COLOR_BLUE, ((uint16_t)time * 60), 2);
		module_cpu_state.u.identify = 1;
		module_identify_time = ((uint16_t)time * 600);
		module_identify_mode = mode;
		module_identify_unlock_time = time == IZIPLUS_IDENTMODE_MAGIC_TIME;
		module_identify_prs = 0;
		State_ResetToggleData_Show();
	}
	else
	{
		State_ClearAttentionInfo();	
		module_identify_unlock_time = 0;
		module_cpu_state.u.identify = 0;
		module_identify_time = 1;
	}
}

const uint8_t CfgSwap_table[][4] = 
{
	{ 0,1,2,3, },	// Chan order: 1234 Value="0" />
	{ 0,1,3,2, },	// Chan order: 1243 Value="1" />
	{ 0,2,1,3, },	// Chan order: 1324 Value="2" />
	{ 0,2,3,1, },	// Chan order: 1342 Value="3" />
	{ 0,3,1,2, },	// Chan order: 1423 Value="4" />
	{ 0,3,2,1, },	// Chan order: 1432 Value="5" />
	{ 1,0,2,3, },	// Chan order: 2134 Value="6" />
	{ 1,0,3,2, },	// Chan order: 2143 Value="7" />
	{ 1,2,0,3, },	// Chan order: 2314 Value="8" />
	{ 1,2,3,0, },	// Chan order: 2341 Value="9" />
	{ 1,3,0,2, },	// Chan order: 2413 Value="10" />
	{ 1,3,2,0, },	// Chan order: 2431 Value="11" />
	{ 2,0,1,3, },	// Chan order: 3124 Value="12" />
	{ 2,0,3,1, },	// Chan order: 3142 Value="13" />
	{ 2,1,0,3, },	// Chan order: 3214 Value="14" />
	{ 2,1,3,0, },	// Chan order: 3241 Value="15" />
	{ 2,3,0,1, },	// Chan order: 3412 Value="16" />
	{ 2,3,1,0, },	// Chan order: 3421 Value="17" />
	{ 3,0,1,2, },	// Chan order: 4123 Value="18" />
	{ 3,0,2,1, },	// Chan order: 4132 Value="19" />
	{ 3,1,0,2, },	// Chan order: 4213 Value="20" />
	{ 3,1,2,0, },	// Chan order: 4231 Value="21" />
	{ 3,2,0,1, },	// Chan order: 4312 Value="22" />
	{ 3,2,1,0, },	// Chan order: 4321 Value="23" />
};

bool IziPlus_Module_HasOutputMaster()
{
	return appConfig->mode == MODE_4_CHANMASTER;
}

bool IziPlus_Module_SingleChannel()
{
	return appConfig->mode == MODE_1_CHANNEL;
}

uint8_t IziPlus_Module_OutputMasterChannel()
{
	return 0;
}

bool IziPlus_Module_Is16bit()
{
	return appConfig->mode == MODE_4_CHANNEL_16;
}

/************************************************************************/
/* chan_idx 0 = Ch1, 1 = Ch2, 2 = C3, 3 = Ch4							*/
/************************************************************************/
uint16_t IziPlus_Module_GetDim(uint8_t chan_idx)
{
	uint8_t swap = CONFIG_PAR_SWAP(appConfig->config);
	const uint8_t* idxs = CfgSwap_table[swap];
	uint16_t val = 0;
	
	int index = IziPlus_Module_SingleChannel() ? 0 : idxs[chan_idx];
	if(IziPlus_Module_HasOutputMaster())
	{
		uint16_t lvl = !IziPlus_Module_IsOverruled() ? Izi_OutputGet(index + 1) : overrule_level_chx[index];
		uint16_t lvl_master = !IziPlus_Module_IsOverruled() ? Izi_OutputGet(0) : 0xFF;				// If so, merge them. If overruled, set master to full
		if(lvl_master != 0xFF || lvl != 0xFF)
			val = lvl_master * lvl;
		else
			val = 0xFFFF;
	}
	else if(IziPlus_Module_Is16bit())
	{
		index *= 2;
		uint16_t lvl = !IziPlus_Module_IsOverruled() ? Izi_OutputGet(index) : overrule_level_chx[index];
		val = Izi_OutputGet(index + 1) | (lvl << 8);
	}
	else
	{
		uint16_t lvl = !IziPlus_Module_IsOverruled() ? Izi_OutputGet(index) : overrule_level_chx[index];
		val = (lvl << 8) | lvl;
	}
	
	iziplus_level_chx[chan_idx] = val;
	
	return val;
}

uint8_t IziPlus_Module_GetChannelAmount()
{
	return module_channels_amount;
}

uint8_t IziPlus_Module_GetChannelAmountTemp()
{
	if(appConfig->mode < MODE_AMOUNT)
		return iziplus_fixdef.modes[appConfig->mode].channels;
	return module_channels_amount;
}

uint8_t IziPlus_Module_GetHwRevision()
{
	return get_hw_id();			
}

uint8_t IziPlus_Module_GetVariant()
{
	return get_device_id();			
}

izi_versionplus_u IziPlus_Module_GetAppVersion()
{
	izi_versionplus_u appversion = { .v.major = firmare_info.version.v.major, .v.minor = firmare_info.version.v.minor, .v.build = firmare_info.version.v.build };
	return appversion;			
}

izi_versionplus_u IziPlus_Module_GetBootVersion()
{
	version_u *boot_version = (version_u *)BOOT_VERSION_ADDRESS;
	izi_versionplus_u bootversion = { .v.major = boot_version->v.major, .v.minor = boot_version->v.minor, .v.build = boot_version->v.build };
	return bootversion;			
}

izi_versionplus_u IziPlus_Module_GetImageVersion()
{
	firmware_t *image_info = (firmware_t *)(FLASH_APP_MIRROR_END - FLASH_APP_FW_INFO_SIZE - FLASH_APP_MIRROR_SHIFT);
	izi_versionplus_u imageversion = { .v.major = image_info->version.v.major, .v.minor = image_info->version.v.minor, .v.build = image_info->version.v.build };
	return imageversion;
}

izi_version_u IziPlus_Module_GetTypedefVersion()
{
	izi_version_u typedefversion = { .v.major = 2, .v.minor = 20 };
	return typedefversion;			// Min version of TDE file
}

izi_version_u IziPlus_Module_GetLedtableVersion()
{
	izi_version_u ledtableversion = { .v.major = 0, .v.minor = 0 };
	return ledtableversion;			// Todo: real led table version
}

uint8_t IziPlus_Module_GetMemoryVersion()
{
	return CONFIG_VERSION;		
}

bool IziPlus_Module_IsOverruled()
{
	return overrule_time_chx > 0;
}

bool IziPlus_Module_Test(uint8_t idx, uint32_t time, uint8_t prio)
{
	if(idx >= MAX_DIM_CHANNELS)
	{
		IziPlus_Module_Overrule_Ch1234(0xFFFFFFFF, time, prio);
		return true;
	}
	IziPlus_Module_Overrule_Ch1234(0x000000FF << (idx * 8), time, prio);
	return false;
}

void IziPlus_Module_Overrule_Chx(uint8_t chan_idx, uint8_t level, uint32_t time, uint8_t prio)
{
	if(prio >= overrule_prio_chx || overrule_time_chx == 0)
	{
		overrule_prio_chx = prio;
		overrule_level_chx[chan_idx] = level;
		overrule_time_chx = time / 100;
		
		//OS_TRACE_ERROR("Overrule channel[%d]: l:%d p:%d t:%d\r\n", chan_idx, level, prio, time);
	}
}

void IziPlus_Module_Overrule_Ch1234(uint32_t dim1234, uint32_t time, uint8_t prio)
{
	if(prio >= overrule_prio_chx || overrule_time_chx == 0)
	{
		overrule_prio_chx = prio;
		for(int i = 0; i < MAX_DIM_CHANNELS; i++)
			overrule_level_chx[i] = (uint8_t)(dim1234 >> (i * 8));
		overrule_time_chx = time / 100;
		//OS_TRACE_ERROR("Overrule channel1234: l:%d p:%d t:%d\r\n", dim1234, prio, time);
	}
}

void IziPlus_NoComCheck()
{
	bool isok = IziPlus_LightDataOk();
	if(!isok)
	{
		if(xTaskGetTickCount() < 5000)				// Do nothing if shorter than 5 sec after power-up (should not go on during discover)
			return;
		
		if((iziplus_data_wasok && (DMXFAIL_OPERATE(appConfig->dmxfail) == DMXFAIL_MAX)) || (!iziplus_data_wasok && (DMXFAIL_PUP(appConfig->dmxfail) == DMXFAIL_MAX)))
		{
			if(iziplus_prs_comcheck == 0)
			{
				for(int i = 0; i < IziPlus_Module_GetChannelAmount(); i++)
				{
					uint8_t current_val = Izi_OutputGet(i);
					if(current_val < 0xFF)
					{
						Izi_OutputSet(IZIOUTPUT_SRC_IZI, i, current_val + 1);	// Set new decreased master value
					}
				}
			}
		}
		else if((iziplus_data_wasok && (DMXFAIL_OPERATE(appConfig->dmxfail) == DMXFAIL_OFF)) || (!iziplus_data_wasok && (DMXFAIL_PUP(appConfig->dmxfail) == DMXFAIL_OFF)))
		{
			if(iziplus_prs_comcheck == 0)
			{
				for(int i = 0; i < IziPlus_Module_GetChannelAmount(); i++)
				{
					uint8_t any_val = Izi_OutputGet(i);
					if(any_val > 0)
						Izi_OutputSetAll(IZIOUTPUT_SRC_IZI, i, any_val - 1);
				}
			}
		}
		
		if(++iziplus_prs_comcheck >= 2)
			iziplus_prs_comcheck = 0;
	}
	else
		iziplus_data_wasok = true;			// Mark it has been ok once since power-up, and so DMXFAIL_OPERATE should be used
	
	iziplus_last_comcheck = isok;
}

/*
 * iziplus_module_def.h
 *
 * Created: 26-2-2022 17:28:25
 *  Author: Milo
 */ 


#ifndef IZIPLUS_MODULE_DEF_H_
#define IZIPLUS_MODULE_DEF_H_

#include "izi_module.h"
#include "iziplus_frames.h"

#define IZIPLUS_DEVTYPE			iziplus_devtype		// IZI-MoodSpot+ HP + emitter offset
#define IZIPLUS_DEVTYPE_BASE	0x220				// CC4+ (is base)
#define IZIPLUS_DEVOFFS_UNKNOWN	30					// Offset for Unknown id of LP

#define MODE_AMOUNT			iziplus_fixdef.max_modes

#define MODE_1_CHANNEL		0
#define MODE_4_CHANMASTER	1
#define MODE_4_CHANNEL		2
#define MODE_SINGLE_CHANNEL	3
#define MODE_TUNABLE_WHITE	4
#define MODE_WARM_DIM		5
#define MODE_CHANGEOVER_TW	6
#define MODE_CHANGEOVER_WD	7
#define MODE_4_CHANNEL_16	8

#define DMXFAIL_OFF			0
#define DMXFAIL_HOLD		1
#define DMXFAIL_MAX			2
#define DMXFAIL_PUP(x)		(x & 0x0F)
#define DMXFAIL_OPERATE(x)	((x >> 4) & 0x0F)

#define CONFIG_AMOUNT		iziplus_fixdef.max_configs //3
#define CONFIG_SIZE			iziplus_config_size		// If 16-bit is used, calc total bytes

#define CONFIG_PAR_CURRENT	0		// Current setting from 150 to 1050mA
#define CONFIG_CURRENT_MA(v)		(150 + (v[0] * 50))		// Convert setting to mA
#define CONFIG_CURRENT_MA_RAW(v)	(v[0])		// Convert setting to mA
#define CONFIG_CURRENT_MA_REV(c, mA)	c[0] = ((mA - 150) / 50)		// Convert mA to setting
#define CONFIG_PAR_SWAP(v)	v[4]

#define CONFIG_FILTER_NORMAL	0
#define CONFIG_FILTER_FAST		1
#define CONFIG_FILTER_DYNAMIC	2
#define CONFIG_FILTER_SLOW		3
#define CONFIG_PAR_FILTER(v)	v[1]

#define CONFIG_MAX_CURVES	4

#define CONFIG_CURVE_LINEAR		0
#define CONFIG_CURVE_SMOOTH		1
#define CONFIG_CURVE_SOFT		2
#define CONFIG_CURVE_EXTRA_SOFT	3
#define CONFIG_PAR_CURVE(v)		v[6]

#define CONFIG_VIRTUAL_IN(v)	(v[5])		// Virtual input (0 = none)

#define MONITOR_AMOUNT		iziplus_fixdef.max_monitors//11
#define MONITOR_SIZE		iziplus_monitor_size		// If 16-bit is used, calc total bytes

#define PRIO_OVERRULE_LOWEST	0
#define PRIO_OVERRULE_LOW		2
#define PRIO_OVERRULE_MEDIUM	4
#define PRIO_OVERRULE_HIGH		6
#define PRIO_OVERRULE_HIGHEST	8

#define MODE_FLASH_AUTO			0x01			// RGBW wins from CCT channel
#define MODE_FLASH_WD_SINGLE	0x02			// Single channel WD (master channel is used as level and tw)
#define MODE_FLASH_DIRECT		0x04			// Uses level direct to output no intelligence
#define MODE_FIRST_16B			0x08			// First 2 channels should be combined as 16 bit
#define MODE_CTRL_NO_RGB		0x10			// When the mode has a control channel, but only WD and TW are valid
#define MODE_FLAG_WWCW			0x20			// WW CW
#define MODE_FLAG_13_24_LINKED	0x40			// 1/3 and 2/4 linked

#define MAX_TEST_LEVELS			4
#define MAX_DIM_CHANNELS		4

#define MAX_MODE_NAME_LEN		28
#define MAX_CFG_NAME_LEN		20

typedef struct
{
	uint8_t dim_outputs;
	uint8_t fe[3];
	uint16_t fw_voltages[4];
	uint32_t test_lvls[MAX_TEST_LEVELS];
}iziplus_emitter_def;

typedef struct
{
	uint8_t channels;
	uint8_t flags;
	uint8_t reserved[2];
	char name[MAX_MODE_NAME_LEN];
	int8_t rgbw_chan;		// Start channel
	int8_t tw_chan;
	int8_t master_chan;
	int8_t control_chan;
}iziplus_mode_def;		// Size  = 36 bytes

typedef struct
{
	uint8_t size;
	uint8_t ref;		// Ref to functionality
	uint8_t idx;		// Index in array to send
	uint8_t reserved;
	uint16_t dflt;
	uint16_t min;
	uint16_t max;
	uint16_t fe;
	char name[MAX_CFG_NAME_LEN];
}iziplus_config_def;	// Size  = 12 bytes

typedef struct
{
	uint8_t size;
	uint8_t ref;		// Ref to functionality
	uint8_t idx;		// Index in array to send
	char name[18];
	uint8_t reserved;	// Size  = 4 bytes
}iziplus_monitor_def;

#define MAX_EMITTER_MODES						12		// Max amount of modes in an emitter
#define MAX_EMITTER_CONFIGS						12		// Max amount of configs in an emitter
#define MAX_EMITTER_MONITORS					24		// Max amount of monitor pars in an emitter
#define MAX_COLOR_ITEMS							17		// Color points to create black body curve
#define MAX_FIX_MODEL_NAME						32

typedef struct
{
	uint8_t max_modes;							// Amount of modes
	uint8_t max_configs;						// Amount of config parameters
	uint8_t max_monitors;						// Amount of monitor parameters
	uint8_t default_mode;						// Default mode
	//uint16_t warn_supply_voltage;				// Calculated on highest forward voltage in emitter, warning given Supply voltage is critical
	//uint16_t min_supply_voltage;				// Calculated on highest forward voltage in emitter, error given Supply voltage is too low (fixture off)
	iziplus_mode_def modes[MAX_EMITTER_MODES];			// Module mode definitions	(4 + 12 * 12 = 148)
	iziplus_config_def configs[MAX_EMITTER_CONFIGS];	// Module config defs		(148 + 12 * 12 = 292)
	iziplus_monitor_def monitors[MAX_EMITTER_MONITORS];	// Module monitor defs		(292 + 4 * 24 = 388)
	char model_name[MAX_FIX_MODEL_NAME];
}iziplus_fixture_def;

typedef struct
{
	uint8_t id;											// Fixture ID offset
	uint8_t version;									// 3 bits major, 5 bits minor
	uint8_t hwref;										// HW ref
	uint8_t max_channels;								// Amount of channels used (max 4 by hardware)
	uint16_t max_current;								// Max current for this emitter (used in LP)
	uint8_t cct_min;									// Minimum CCT color value (Kelvin/100, example 14 = 1400K)
	uint8_t cct_max;									// Maximum CCT color value (Kelvin/100, example 65 = 6500K)
	uint32_t test_lvls[MAX_DIM_CHANNELS];				// Test levels directly on hardware by switch (amount used is set by max_channels) (24)
	uint8_t patch_rgbw[MAX_DIM_CHANNELS];					// RGBW patch_rgbw				(28)
	uint8_t bbc_chx[MAX_DIM_CHANNELS][MAX_COLOR_ITEMS + 3];			// Black body curve Red color intensity (45) +3 is for better alignment, last 3 bytes per row may be used for something else in the future
	uint16_t fw_voltage_tables[MAX_DIM_CHANNELS];		// Forward voltages of LEDs per channel (in patched order) (116)
	uint8_t fw_minvoltage_tables[MAX_DIM_CHANNELS];		// Forward voltages of LEDs per channel (in patched order) (120)
	int16_t corr_temp_tables[MAX_DIM_CHANNELS];			// Temp correction (128)
	uint16_t corr_tjc_tables[MAX_DIM_CHANNELS];			// Temp correction (136)
	uint32_t max_power;									// Max power for power limiter (140)
	uint16_t tw_gain;									// 70% in TW mode, so the 10W is not reached over the complete range (142)
	uint16_t wd_gain;									// 86% in WD mode, so the 10W is not reached over the complete range (144)
	uint8_t rgbw_sensitivity[MAX_DIM_CHANNELS];			// { 255, 184, 255, 102 };  // RGBW Sensitivity (in patched order!) (148)
	uint16_t max_current_ch[MAX_DIM_CHANNELS];			// Max current per channel (156)
	uint8_t reserved[84];
	uint16_t device_type;								// The base device type it has to match
	uint16_t dac_min;									// Future functionality if dac_min should be overruled by emitter (0 = don't use)
	char chan_chars[MAX_DIM_CHANNELS];					// Character per channel
	uint32_t indicate_lvl;								// Level of all channels when identifying or when Dmx fail to Full
	uint8_t variant;									// Set if emitter must match with a variant of the base pcb, (0 = match all, 1 .. must match variant)
	uint8_t options;									// Future options
	uint16_t crc;										// CRC over emitter fixture mem (256)
}emitter_fixture_def;

//extern const iziplus_mode_def module_modes[MODE_AMOUNT];
//extern const iziplus_config_def module_config[CONFIG_AMOUNT];
//extern const uint8_t channel_patch[4];
//extern const uint16_t FwVoltageTables[4];
//extern const uint16_t MaxCurrent;
//extern const uint16_t MaxPower;

extern iziplus_fixture_def	iziplus_fixdef;
extern bool emitter_fixdef_ok;
extern uint16_t  iziplus_config_size;
extern uint16_t  iziplus_monitor_size;
extern uint16_t iziplus_devtype, iziplus_devtype_base; 
extern int16_t iziplus_emitter_state;
extern uint32_t iziplus_maxpower; 

// Proto
bool IziPlus_Module_Init();
bool IziPlus_Module_InitReady();
void IziPlus_Module_Update();
void IziPlus_Module_Timer100ms();
void IziPlus_Module_SetIdentify(uint8_t mode, uint8_t time);
iziplus_cpustate_u IziPlus_Module_GetCpuState();
uint8_t *IziPlus_Module_GetMonitor();
uint8_t IziPlus_Module_GetChannelAmount();
uint8_t IziPlus_Module_GetChannelAmountTemp();
uint8_t IziPlus_Module_GetHwRevision();
uint8_t IziPlus_Module_GetVariant();
izi_versionplus_u IziPlus_Module_GetAppVersion();
izi_versionplus_u IziPlus_Module_GetBootVersion();
izi_versionplus_u IziPlus_Module_GetImageVersion();
izi_version_u IziPlus_Module_GetTypedefVersion();
izi_version_u IziPlus_Module_GetLedtableVersion();
uint8_t IziPlus_Module_GetMemoryVersion();
bool IziPlus_Module_IsOverruled();
void IziPlus_Module_Overrule_Chx(uint8_t chan_idx, uint8_t level, uint32_t time, uint8_t prio);
void IziPlus_Module_Overrule_Ch1234(uint32_t dim1234, uint32_t time, uint8_t prio);
bool IziPlus_Module_IsIdentifying();
bool IziPlus_Module_IsModeSwitching();
void IziPlus_NoComCheck();
void IziPlus_Module_IdentifySync();
uint16_t IziPlus_Module_GetDim(uint8_t chan_idx);
uint32_t IziPlus_Module_GetTable(uint8_t chan_idx, uint32_t val);
bool IziPlus_Module_Test(uint8_t idx, uint32_t time, uint8_t prio);
uint32_t IziPlus_Module_TempLimit(uint32_t level);
bool IziPlus_Module_PowerCheck();
uint8_t IziPlus_Module_MonitorValue(uint8_t ref, uint8_t *bfr);
uint8_t IziPlus_Module_ConfigValue(uint8_t ref, uint8_t *bfr);
uint8_t *IziPlus_Module_GetConfig();
uint8_t IziPlus_Module_SetConfigValue(uint8_t ref, uint8_t *bfr, uint8_t len);
uint8_t IziPlus_Module_SetConfigByIndex(uint8_t idx, uint8_t *bfr, uint8_t len);

bool IziPlus_Module_IsSafetyOff_Ch1();
bool IziPlus_Module_IsSafetyOff_Ch2();
bool IziPlus_Module_IsSafetyOff_Ch3();
bool IziPlus_Module_IsSafetyOff_Ch4();
bool IziPlus_Module_IsSafetyOff_Chx(uint8_t idx);
void IziPlus_Module_SafetyOff_Chx(uint8_t idx, uint16_t time);

#endif /* IZIPLUS_MODULE_DEF_H_ */
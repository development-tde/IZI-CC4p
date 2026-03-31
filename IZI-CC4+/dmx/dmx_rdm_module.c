/*
 * dmx_rdm_module.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Milo
 */

#include "includes.h"
#include "dmx_rdm_models.h"
#include "dmx_rdm_module.h"
#include "rdm.h"
#include "esta.h"
#include "version.h"
#include "izi_input.h"
#include "appconfig.h"
#include "iziplus_module_def.h"
#include "stepdown.h"

// Defines
#define MAX_SENSORDEF		4			// Can not exceed 16 due to 16-bit sensor_flags
const SENSORDEF dmx_rdm_sensors[MAX_SENSORDEF] =
{
	// Sensor number is overwritten when requested
	{ .sensorNumber = 0, .sensorType = E120_SENS_VOLTAGE, .units = E120_UNITS_VOLTS_DC, .prefix = E120_PREFIX_DECI, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(560), .normalMin = SWAPINT(400), .normalMax = SWAPINT(520), .recordedValueSupport = 0, .description = "Supply voltage"   },
	{ .sensorNumber = 1, .sensorType = E120_SENS_POWER, .units = E120_UNITS_WATT, .prefix = E120_PREFIX_DECI, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1000), .normalMin = SWAPINT(0), .normalMax = SWAPINT(700), .recordedValueSupport = 0, .description = "Power"   },
	{ .sensorNumber = 2, .sensorType = E120_SENS_TEMPERATURE, .units = E120_UNITS_CENTIGRADE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(128), .normalMin = SWAPINT(15), .normalMax = SWAPINT(90), .recordedValueSupport = 0, .description = "Intern temp"  },
	{ .sensorNumber = 3, .sensorType = E120_SENS_TEMPERATURE, .units = E120_UNITS_CENTIGRADE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(128), .normalMin = SWAPINT(15), .normalMax = SWAPINT(100), .recordedValueSupport = 0, .description = "LED temp"  },
	/*{ .sensorNumber = 4, .sensorType = E120_SENS_POWER, .units = E120_UNITS_WATT, .prefix = E120_PREFIX_DECI, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1000), .normalMin = SWAPINT(0), .normalMax = SWAPINT(750), .recordedValueSupport = 0, .description = "Power"   },
	{ .sensorNumber = 5, .sensorType = E120_SENS_CONTACTS, .units = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1), .normalMin = SWAPINT(0), .normalMax = SWAPINT(1), .recordedValueSupport = 0, .description = "Contact 1"   },
	{ .sensorNumber = 6, .sensorType = E120_SENS_CONTACTS, .units = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1), .normalMin = SWAPINT(0), .normalMax = SWAPINT(1), .recordedValueSupport = 0, .description = "Contact 2"   },
	{ .sensorNumber = 7, .sensorType = E120_SENS_CONTACTS, .units = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1), .normalMin = SWAPINT(0), .normalMax = SWAPINT(1), .recordedValueSupport = 0, .description = "Contact 3"   },
	{ .sensorNumber = 8, .sensorType = E120_SENS_CONTACTS, .units = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .rangeMin = SWAPINT(0), .rangeMax = SWAPINT(1), .normalMin = SWAPINT(0), .normalMax = SWAPINT(10), .recordedValueSupport = 0, .description = "Contact 4"   }
	// If changed check dmx_rdm_sensor_refs, uses same index
	*/
};

typedef struct rdm_sensor_ref_s
{
	uint16_t 	refid;
	uint16_t 	mask;
	uint8_t     mul;
	uint8_t		div;
	uint8_t 	size;
	uint8_t		rdm_flags;
}rdm_sensor_ref_t;

#define DMX_RDM_SENSOR_FLAGS_NONFIXTURE_ONLY	1

const rdm_sensor_ref_t dmx_rdm_sensor_refs[MAX_SENSORDEF] =
{
	{ .refid =	MONTYPE_REFID_SUPPLY_VOLT, .mul = 10, .div = 4, .size = 1, .mask = 0xFFFF, .rdm_flags = 0 },
	{ .refid =	MONTYPE_REFID_POWER3, .mul = 10, .div = 3, .size = 1, .mask = 0xFFFF, .rdm_flags = 0 },
	{ .refid =	MONTYPE_REFID_INTERN_TEMP, .mul = 1, .div = 1, .size = 1, .mask = 0xFFFF, .rdm_flags = 0 },
	{ .refid =	MONTYPE_REFID_LED_TEMP, .mul = 1, .div = 1, .size = 1, .mask = 0xFFFF, .rdm_flags = 0 },
	/*{ .refid =	MONTYPE_REFID_ACT_CONTACTS, .mul = 1, .div = 1, .mask = 0x0001, .size = 1, .rdm_flags = 0 },
	{ .refid =	MONTYPE_REFID_ACT_CONTACTS, .mul = 1, .div = 1, .mask = 0x0002, .size = 1, .rdm_flags = DMX_RDM_SENSOR_FLAGS_NONFIXTURE_ONLY },
	{ .refid =	MONTYPE_REFID_ACT_CONTACTS, .mul = 1, .div = 1, .mask = 0x0004, .size = 1, .rdm_flags = DMX_RDM_SENSOR_FLAGS_NONFIXTURE_ONLY },
	{ .refid =	MONTYPE_REFID_ACT_CONTACTS, .mul = 1, .div = 1, .mask = 0x0008, .size = 1, .rdm_flags = DMX_RDM_SENSOR_FLAGS_NONFIXTURE_ONLY }*/
};

#define MAX_CUSTOM_PIDS		9
const PARAMETER_DESCRIPTION dmx_rdm_custom_pids[MAX_CUSTOM_PIDS] =
{
	{ .pid = SWAPINT(E120_TDE_MAX_OUTPUT), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(100), .defaultValue = SWAPINT32(100), .description = "Maximum Output %" },
	//{ .pid = SWAPINT(E120_TDE_COLOR_RANGE), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(1), .defaultValue = SWAPINT32(0), .description = "CCT Range Max (0=4000K 1=6500K)" },
	{ .pid = SWAPINT(E120_TDE_FILTER_MODE), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(3), .defaultValue = SWAPINT32(0), .description = "Filter mode" },
	{ .pid = SWAPINT(E120_TDE_DIM_CURVE), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(3), .defaultValue = SWAPINT32(0), .description = "Dim curve" },
	{ .pid = SWAPINT(E120_TDE_CCT_TW_MIN), .dataType = E120_DS_UNSIGNED_WORD, .pdlSize = 2, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(1000), .maxValidValue = SWAPINT32(2700), .defaultValue = SWAPINT32(1400), .description = "TW CCT Min Kelvin" },
	{ .pid = SWAPINT(E120_TDE_CCT_TW_MAX), .dataType = E120_DS_UNSIGNED_WORD, .pdlSize = 2, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(2700), .maxValidValue = SWAPINT32(6500), .defaultValue = SWAPINT32(4000), .description = "TW CCT Max Kelvin" },
	{ .pid = SWAPINT(E120_TDE_CCT_WD_MIN), .dataType = E120_DS_UNSIGNED_WORD, .pdlSize = 2, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(1000), .maxValidValue = SWAPINT32(2700), .defaultValue = SWAPINT32(1400), .description = "WD CCT min Kelvin" },
	{ .pid = SWAPINT(E120_TDE_CCT_WD_MAX), .dataType = E120_DS_UNSIGNED_WORD, .pdlSize = 2, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(2700), .maxValidValue = SWAPINT32(6500), .defaultValue = SWAPINT32(3000), .description = "WD CCT max Kelvin" },
	{ .pid = SWAPINT(E120_TDE_DMXFAIL_PUP), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(4), .defaultValue = SWAPINT32(0), .description = "DMX Fail Power-up" },
	{ .pid = SWAPINT(E120_TDE_DMXFAIL_OPERATION), .dataType = E120_DS_UNSIGNED_BYTE, .pdlSize = 1, .commandClass = E120_CC_GET_SET, .type = 0, .unit = E120_UNITS_NONE, .prefix = E120_PREFIX_NONE, .minValidValue = SWAPINT32(0), .maxValidValue = SWAPINT32(4), .defaultValue = SWAPINT32(0), .description = "DMX Fail Operation" },
	// If changed check dmx_rdm_custom_pids_refids
};

typedef struct rdm_cfg_ref_s
{
	uint16_t 	refid;
	uint16_t 	offset;
	uint8_t     mul;
	uint8_t		div;
	uint8_t 	fe;
	uint8_t		fe2;
}rdm_cfg_ref_t;

const rdm_cfg_ref_t dmx_rdm_custom_pids_refids[MAX_CUSTOM_PIDS] =
{
	{ .refid = CFGTYPE_REFID_MAXOUTPUT, .offset = 0, .mul = 1, .div = 1 },
	//{ .refid = CFGTYPE_REFID_TTW_CCT_COL, .offset = 0, .mul = 1, .div = 1 },
	{ .refid = CFGTYPE_REFID_FILTER_MODE, .offset = 0, .mul = 1, .div = 1 },
	{ .refid = CFGTYPE_REFID_DIM_CURVE, .offset = 0, .mul = 1, .div = 1 },
	{ .refid = CFGTYPE_REFID_TW_CCT_MIN, .offset = 0, .mul = 100, .div = 1 },
	{ .refid = CFGTYPE_REFID_TW_CCT_MAX, .offset = 0, .mul = 100, .div = 1 },
	{ .refid = CFGTYPE_REFID_WD_CCT_MIN, .offset = 0, .mul = 100, .div = 1 },
	{ .refid = CFGTYPE_REFID_WD_CCT_MAX, .offset = 0, .mul = 100, .div = 1 },
	{ .refid = CFGTYPE_REFID_DMXFAIL_PUP, .offset = 0, .mul = 1, .div = 1 },
	{ .refid = CFGTYPE_REFID_DMXFAIL_OPERATION, .offset = 0, .mul = 1, .div = 1 },
	
};

#define TASK_DMXRDM_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_DMXRDM_TASK_PRIORITY (tskIDLE_PRIORITY + 3)

#define RDM_DELAY_APPCFG_WRITE		0x01
#define RDM_DELAY_RESET				0x02

// Globals
static uint16_t								rdm_modules_amount = 0;
static uint16_t								rdm_delayed_request = 0;
static uint16_t								rdm_flags;
static uint8_t								rdm_queue_size = 0;
static uint8_t								rdm_queue_head = 0;
static uint8_t								rdm_queue_tail = 0;
static rdm_module_queue_t					rdm_queue[MAX_QUEUED_MESSAGES];

// Proto
void DmxRdm_Module_Swap(rdm_module_t *a, rdm_module_t *b);
void DmxRdm_Module_Sort(uint8_t start);


/**
 * Init rdm modules
 */
void DmxRdm_Module_Init()
{
	rdm_flags = 0;
}



void DmxRdm_Module_Delayed()
{
	if(rdm_delayed_request & RDM_DELAY_APPCFG_WRITE)
	{
		appconfig_t *appconfig = AppConfig_GetPtr();				// Get current app config in RAM
		AppConfig_Set(appconfig);						// Write back

		rdm_delayed_request &= ~RDM_DELAY_APPCFG_WRITE;
	}
	if(rdm_delayed_request & RDM_DELAY_RESET)
	{
		StepDown_Close(1000);
		vTaskDelay(200);
		AppLog_Set();
		NVIC_SystemReset();
	}
}

uint8_t dmxrdm_update_data_bfr[2];
uint8_t dmxrdm_update_prs = 0;
uint8_t dmxrdm_update_max = 0;

void DmxRdm_Module_Timer()
{
	if(StepDown_SleepCheck())
	{
		if(dmxrdm_update_max < 20)
			dmxrdm_update_max += 2;			// If off, goto 20 second update
	}
	else
		dmxrdm_update_max = 4;				// If on, report every 4 seconds
	
	if(++dmxrdm_update_prs >= dmxrdm_update_max)			// New update? 
	{
		for(uint8_t idx = 0; idx < MAX_SENSORDEF; idx++)	// Update all at once
		{
			dmxrdm_update_data_bfr[0] = idx;
			DmxRdm_Module_AddQueuedMessage(0, 0, E120_SENSOR_VALUE, dmxrdm_update_data_bfr, 1);
		}
		dmxrdm_update_prs = 0;
	}
}

/**
 * @brief Get the first device id of module that is unmuted
 */
int16_t DmxRdm_Module_GetUnmuted()
{
	return rdm_flags & DMX_RDM_FLAG_MUTE ? -1 : 0;
}

/**
 * @brief Fill the given device id
 */
bool DmxRdm_Module_GetDeviceID(int16_t idx, DEVICEID dev_id)
{
	dev_id[0] = ESTA_MANUFACTURER_ID_H;
	dev_id[1] = ESTA_MANUFACTURER_ID_L;
	dev_id[2] = (uint8_t)(PRODUCTION_SERIAL >> 24);
	dev_id[3] = (uint8_t)(PRODUCTION_SERIAL >> 16);
	dev_id[4] = (uint8_t)(PRODUCTION_SERIAL >> 8);
	dev_id[5] = (uint8_t)(PRODUCTION_SERIAL >> 0);

	return true;
}

/**
 * Get the supported parameters for this device
 */
uint8_t DmxRdm_Module_GetSupportedParameters(int16_t idx, uint16_t sub_idx, uint16_t *parameters, uint8_t max_parameters)
{
	if(idx == 0)
	{
		uint8_t amount = 0;
		
		if(max_parameters < 10)			// Not even default parameters fit?
			return 0;

		parameters[amount++] = E120_DEVICE_MODEL_DESCRIPTION;
		parameters[amount++] = E120_MANUFACTURER_LABEL;
		parameters[amount++] = E120_DEVICE_LABEL;
		parameters[amount++] = E120_DMX_PERSONALITY;
		parameters[amount++] = E120_DMX_PERSONALITY_DESCRIPTION;
		parameters[amount++] = E120_LAMP_STATE;
		parameters[amount++] = E120_BOOT_SOFTWARE_VERSION_LABEL;
		parameters[amount++] = E120_RESET_DEVICE;
		parameters[amount++] = E120_QUEUED_MESSAGE;
		parameters[amount++] = E120_STATUS_MESSAGES;
		parameters[amount++] = E120_STATUS_ID_DESCRIPTION;
		if((amount + 2 < max_parameters) && MAX_SENSORDEF > 0)
		{
			parameters[amount++] = E120_SENSOR_DEFINITION;
			parameters[amount++] = E120_SENSOR_VALUE;
		}
		parameters[amount++] = E120_DEVICE_HOURS;
		parameters[amount++] = E120_LAMP_HOURS;

		for(int c = 0; c < MAX_CUSTOM_PIDS; c++)			// Search for all custom PIDs known
		{
			if((amount + 1) < max_parameters)
				parameters[amount++] = SWAPINT(dmx_rdm_custom_pids[c].pid);
		}

		//OS_TRACE_ERROR("Parameters: %d (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)", amount, parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);

		return amount;
	}
	return 0;
}

/**
 * Get the description parameters for this device
 */
PARAMETER_DESCRIPTION *DmxRdm_Module_GetDescriptionParameters(int16_t idx, uint16_t sub_idx, uint16_t pid)
{
	if(idx == 0)
	{
		for(int i = 0; i < MAX_CUSTOM_PIDS; i++)
		{
			if(dmx_rdm_custom_pids[i].pid == SWAPINT(pid))
			{
				return (PARAMETER_DESCRIPTION *)&dmx_rdm_custom_pids[i];
			}
		}
	}
	return NULL;
}


void DmxRdm_Module_Mute(bool mute, int16_t idx)
{
	if(mute)
		rdm_flags |= DMX_RDM_FLAG_MUTE;
	else
		rdm_flags &= ~DMX_RDM_FLAG_MUTE;
}

int16_t DmxRdm_Module_SearchSerial(uint32_t serial, uint8_t *sub_amount)
{
	if(PRODUCTION_SERIAL == serial)
	{
		if(sub_amount != NULL)
			*sub_amount = 0;
		return 0;
	}
	return -1;
}

bool DmxRdm_Module_GetDeviceInfo(int16_t idx, uint16_t sub_idx, DEVICEINFO *devInfo)
{
	if(idx == 0)
	{
		devInfo->protocolMajor = 1;
		devInfo->protocolMinor = 0;
		devInfo->deviceModel = SWAPINT(iziplus_devtype);
		devInfo->productCategory = SWAPINT(E120_PRODUCT_CATEGORY_FIXTURE);		// In the RDM protocol, "projector" is a term that refers to devices that "project" light (light fixtures).
		devInfo->softwareVersion = SWAPINT32(firmare_info.version.all);
		devInfo->personalityCount = iziplus_fixdef.max_modes;
		devInfo->footprint = appConfig->mode < iziplus_fixdef.max_modes ? SWAPINT(iziplus_fixdef.modes[appConfig->mode].channels) : 0;			// Todo: check mode!! Also check E120_DMX_START_ADDRESS in E120_DMX_START_ADDRESS
		devInfo->currentPersonality = appConfig->mode + 1;						// Start at 1 (not 0)
		devInfo->startAddress = SWAPINT((appConfig->channel + 1));
		
		devInfo->subDeviceCount = SWAPINT(0);
		devInfo->sensorCount = MAX_SENSORDEF;
		
		OS_TRACE_ERROR("Device Info[%d]: %d %d", idx, devInfo->personalityCount, devInfo->softwareVersion);

		return true;
	}
	return false;
}

uint8_t DmxRdm_Module_GetModelName(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len)
{
	if(idx == 0)
	{
		strncpy(name, iziplus_fixdef.model_name, max_len);
		return strlen(name);
	}
	return 0;
}

uint8_t DmxRdm_Module_GetLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len)
{
	if(idx == 0)
	{
		strncpy(name, appConfig->name, max_len);
		return strlen(name);
	}
	return 0;
}

bool DmxRdm_Module_SetLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t len)
{
	if(idx == 0)
	{
		appconfig_t *appconfig = AppConfig_Get();							// Get current settings in ram
		strncpy(appconfig->name, name, len);								// Change text
		
		rdm_delayed_request |= RDM_DELAY_APPCFG_WRITE;
		
		return true;
	}
	return false;
}

bool DmxRdm_Module_SetIdentify(int16_t idx, uint16_t sub_idx, bool on)
{
	if(idx == 0|| idx == DMX_RDM_BC_INDEX)
	{
		if(on)
		{
			IziPlus_Module_SetIdentify(IZIPLUS_IDENTMODE_BLINK, 5);
			rdm_flags |= DMX_RDM_FLAG_IDENTIFY;
		}
		else
		{
			IziPlus_Module_SetIdentify(IZIPLUS_IDENTMODE_BLINK, 0);
			rdm_flags &= ~DMX_RDM_FLAG_IDENTIFY;
		}

		return true;
	}
	return false;
}

bool DmxRdm_Module_GetIdentify(int16_t idx, uint16_t sub_idx)
{
	if(idx == 0)
	{
		return (rdm_flags & DMX_RDM_FLAG_IDENTIFY) > 0;
	}
	return false;
}

bool DmxRdm_Module_SetReset(int16_t idx, uint16_t sub_idx, uint8_t type)
{
	if(idx == 0)
	{
		rdm_delayed_request |= RDM_DELAY_RESET;
		
		return true;
	}
	return false;
}

/**
 * Set address (address given starts from 0)
 */
bool DmxRdm_Module_SetAddress(int16_t idx, uint16_t sub_idx, uint16_t address)
{
	if(idx == 0)
	{
		uint16_t footprint = appConfig->mode < iziplus_fixdef.max_modes ? iziplus_fixdef.modes[appConfig->mode].channels : 0;
		if((address + footprint) < 1024 && ((address / 512) == ((address + footprint)/512)))		// Within 2 universes and on 2 universes?
		{
			appconfig_t *appconfig = AppConfig_Get();							// Get current settings in ram
			appconfig->channel = address;							// Change text
		
			rdm_delayed_request |= RDM_DELAY_APPCFG_WRITE;
			
			return true;
		}
	}
	return false;
}

/**
 * Get address (starting from 0)
 */
uint16_t DmxRdm_Module_GetAddress(int16_t idx, uint16_t sub_idx)
{
	if(idx == 0)
	{
		return appConfig->channel;
	}
	return 0;
}

uint8_t DmxRdm_Module_GetVersionLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len, bool boot)
{
	if(idx == 0)
	{
		if(boot)
		{
			version_u *boot_version = (version_u *)BOOT_VERSION_ADDRESS;
			sprintf(name, "%d.%d.%d", boot_version->v.major, boot_version->v.minor, boot_version->v.build);
		}
		else
			sprintf(name, "%d.%d.%d", firmare_info.version.v.major, firmare_info.version.v.minor, firmare_info.version.v.build);
		
		return strlen(name);
	}
	return 0;
}

/**
 * Get name of given personality (start from 0) and the number of channels, returns the length of the mode string. If 0 is returned the personality does not exist
 */
uint8_t DmxRdm_Module_GetPersonalityDescription(int16_t idx, uint16_t sub_idx, uint8_t pers, char* name, uint8_t max_len, uint8_t *channels)
{
	if(idx == 0)
	{
		if(pers < iziplus_fixdef.max_modes)
		{
			strncpy(name, iziplus_fixdef.modes[pers].name, MAX_MODE_NAME_LEN);
			if(channels != NULL)
				*channels = iziplus_fixdef.modes[pers].channels;
			return strlen(name);
		}
	}
	return 0;
}

/**
 * Set personality (pers starts from 0)
 */
bool DmxRdm_Module_SetPersonality(int16_t idx, uint16_t sub_idx, uint8_t pers)
{
	if(idx == 0)
	{
		if(pers < iziplus_fixdef.max_modes)
		{
			uint16_t footprint = iziplus_fixdef.modes[pers].channels;
			if((appConfig->channel + footprint) < 1024 && ((appConfig->channel / 512) == ((appConfig->channel + footprint)/512)))		// Within 2 universes and on 2 universes?
			{
				appconfig_t *appconfig = AppConfig_Get();							// Get current settings in ram
				appconfig->mode = pers;							// Change mode
				
				//rdm_delayed_request |= RDM_DELAY_APPCFG_WRITE;
				
				return true;
			}
		}
	}
	return false;
}

/**
 * Get personality (starting from 0)
 */
int8_t DmxRdm_Module_GetPersonality(int16_t idx, uint16_t sub_idx, uint8_t *max_pers)
{
	if(idx == 0)
	{
		return appConfig->mode;
	}
	return -3;
}

/**
 * Set lamp state
 */
bool DmxRdm_Module_SetLampState(int16_t idx, uint16_t sub_idx, uint8_t state)
{
	if(idx == 0)
	{
		if(state == E120_LAMP_STRIKE)
		{
			IziPlus_Module_SetIdentify(IZIPLUS_IDENTMODE_SPOTON, 5);
			rdm_flags |= DMX_RDM_FLAG_IDENTIFY;
		}
		else if(state == E120_LAMP_OFF)
		{
			IziPlus_Module_SetIdentify(IZIPLUS_IDENTMODE_SPOTON, 5);
			rdm_flags &= ~DMX_RDM_FLAG_IDENTIFY;
		}
		return true;
	}
	return false;
}

/**
 * Get lamp stat
 */
uint8_t DmxRdm_Module_GetLampState(int16_t idx, uint16_t sub_idx)
{
	if(idx >= 0 && idx < rdm_modules_amount)
	{
		int8_t error = State_GetHighestError();
		if(error == STATE_ERROR_NO_EMITTER)
			return E120_LAMP_NOT_PRESENT;
		else if(error == STATE_ERROR_EMITTER_DATA || error == STATE_ERROR_EMITTER_TYPE)
			return E120_LAMP_ERROR;
		
		return E120_LAMP_ON;
	}
	return E120_LAMP_OFF;
}

/**
 * Get device hours (returns 0 if no found)
 */
uint32_t DmxRdm_Module_GetDeviceHours(int16_t idx, uint16_t sub_idx)
{
	if(idx == 0)
	{	
		return (appLogData->active_sec / (60 * 60));
	}
	return 0;
}

/**
 * Get burn hours (returns 0 if no found)
 */
uint32_t DmxRdm_Module_GetBurnHours(int16_t idx, uint16_t sub_idx)
{
	if(idx == 0)
	{
		return (appLogData->operating_sec / (60 * 60));
	}
	return 0;
}

volatile SENSORDEF dmx_rdm_sensor_def;
/**
 * Search for sensor definition
 */
SENSORDEF* DmxRdm_Module_GetSensorDef(int16_t idx, uint16_t sub_idx, uint8_t sensor_idx)
{
	if(idx == 0)
	{
		if(sensor_idx < MAX_SENSORDEF)
		{
			memcpy((uint8_t *)&dmx_rdm_sensor_def, &dmx_rdm_sensors[sensor_idx], sizeof(SENSORDEF));
			if(dmx_rdm_sensor_refs[sensor_idx].refid == MONTYPE_REFID_POWER3)
			{
				SENSORDEF *sensor_def = (SENSORDEF *)&dmx_rdm_sensor_def;
				sensor_def->rangeMax = SWAPINT(iziplus_maxpower > 50000 ? 1000 : 500);
				sensor_def->normalMax = SWAPINT(iziplus_maxpower / 100);
			}
			OS_TRACE_INFO("Sensor def[%d]: %d %s\r\n", idx, sensor_idx, dmx_rdm_sensor_def.description);
			return (SENSORDEF*)&dmx_rdm_sensor_def;
		}
	}
	return NULL;
}

uint32_t DmxRdm_Module_GetSensorValue(int16_t idx, uint16_t sub_idx, uint8_t sensor_idx, uint16_t *data)
{
	if(idx == 0)
	{
		if(sensor_idx < MAX_SENSORDEF)
		{
			OS_TRACE_INFO("Sensor1[%d]: %d\r\n", idx, sensor_idx);
			
			uint8_t bfr[4];
			uint8_t size = IziPlus_Module_MonitorValue(dmx_rdm_sensor_refs[sensor_idx].refid, bfr);
			if(size == 2)
			{
				uint16_t value = ((bfr[0] << 0) + (bfr[1] << 8)) & dmx_rdm_sensor_refs[sensor_idx].mask;
				value = (value * dmx_rdm_sensor_refs[sensor_idx].mul + dmx_rdm_sensor_refs[sensor_idx].div / 2) / dmx_rdm_sensor_refs[sensor_idx].div;
				*data = (uint16_t)value;
				return 2;
			}
			else if(size == 1)
			{
				*data = (uint16_t)((bfr[0] & dmx_rdm_sensor_refs[sensor_idx].mask) * dmx_rdm_sensor_refs[sensor_idx].mul + dmx_rdm_sensor_refs[sensor_idx].div / 2) / dmx_rdm_sensor_refs[sensor_idx].div;
				return 1;
			}
		}
	}

	return 0;
}

/**
 * Set manufacturer specific data
 */
bool DmxRdm_Module_SetTdeSpecific(int16_t idx, uint16_t sub_idx, uint16_t pid, uint16_t data)
{
	if(idx == 0)
	{
		for(int c = 0; c < MAX_CUSTOM_PIDS; c++)			// Search for all custom PIDs known
		{
			if(pid == SWAPINT(dmx_rdm_custom_pids[c].pid))
			{
				OS_TRACE_INFO("TDE set[%d]: 0x%x %d (%d)\r\n", idx, pid, data, SWAPINT32(dmx_rdm_custom_pids[c].minValidValue));
				if((data < (SWAPINT32(dmx_rdm_custom_pids[c].minValidValue))) || (data > (SWAPINT32(dmx_rdm_custom_pids[c].maxValidValue))))
					return false;

				data = ((data - dmx_rdm_custom_pids_refids[c].offset) * dmx_rdm_custom_pids_refids[c].div) / dmx_rdm_custom_pids_refids[c].mul;
				IziPlus_Module_SetConfigValue(dmx_rdm_custom_pids_refids[c].refid, (uint8_t *)&data, dmx_rdm_custom_pids[c].pdlSize);
				
				rdm_delayed_request |= RDM_DELAY_APPCFG_WRITE;
				
				return true;							// All set OK
			}
		}
	}
	return false;
}

/**
 * Get manufacturer specific data
 */
int16_t DmxRdm_Module_GetTdeSpecific(int16_t idx, uint16_t sub_idx, uint16_t pid, uint16_t *data)
{
	if(idx == 0)
	{
		for(int c = 0; c < MAX_CUSTOM_PIDS; c++)			// Search for all custom PIDs known
		{
			if(pid == SWAPINT(dmx_rdm_custom_pids[c].pid))
			{
				uint8_t bfr[4];
				uint8_t amount = IziPlus_Module_ConfigValue(dmx_rdm_custom_pids_refids[c].refid, bfr);
				if(amount > 0)
				{
					uint16_t new_data = ((bfr[0] * dmx_rdm_custom_pids_refids[c].mul) / dmx_rdm_custom_pids_refids[c].div) + dmx_rdm_custom_pids_refids[c].offset;
					*data = new_data;
					return dmx_rdm_custom_pids[c].pdlSize;
				}
			}
		}
	}
	return 0;
}

/**
 * Adds a message to the queue
 * Returns 1 if successful, 0 if the queue is full.
 */
uint8_t DmxRdm_Module_AddQueuedMessage(int16_t idx, uint16_t sub_idx, uint16_t pid, const uint8_t* data, uint8_t data_length)
{
	if(idx == 0)
	{
		if (rdm_queue_size >= MAX_QUEUED_MESSAGES) {
			return 0; // Queue is full
		}

		for(int i = 0; i < MAX_QUEUED_MESSAGES; i++)
		{
			if(rdm_queue[i].pid == pid && (data_length == 0 || data[0] == rdm_queue[i].data))			// Already pending? Check PID and first byte
				return 0;
		}

		uint8_t tail = rdm_queue_tail;
		// Add message at queueTail position
		rdm_queue[tail].pid = pid;
		rdm_queue[tail].length = data_length;
		if(data_length > 0 && data != NULL)
			rdm_queue[tail].data = data[0];

		//OS_TRACE_INFO("Queue PID set: 0x%04X (%d) @ %d/%d\r\n", pid, data_length, tail, rdm_modules[idx].queueSize + 1);

		// Update queue pointers
		rdm_queue_tail = (tail + 1) % MAX_QUEUED_MESSAGES;
		rdm_queue_size++;

		return 1; // Success
	}
	return 0;
}

/**
 * Retrieves the next queued message
 * Returns 1 if a message was available, 0 if the queue is empty.
 */
uint8_t DmxRdm_Module_GetQueuedMessage(uint16_t idx, uint16_t sub_idx, uint16_t* pid, uint8_t* data, uint8_t* data_length, uint8_t *queue_len)
{
	if(idx == 0)
	{
		if (rdm_queue_size == 0) {
			return 0; // Nu messages in queue
		}

		uint8_t head = rdm_queue_head;
		// Retrieve the message at queueHead position
		*pid = rdm_queue[head].pid;
		*data_length = rdm_queue[head].length;
		if(*data_length > 0)
			data[0] = rdm_queue[head].data;

		rdm_queue[head].pid = 0;					// Set to 0 for faster search in add
		// Remove message from queue (move head forward)
		rdm_queue_head = (head + 1) % MAX_QUEUED_MESSAGES;
		rdm_queue_size--;

		*queue_len = rdm_queue_size;

		OS_TRACE_INFO("Queue PID get: 0x%04X (%d) @ %d/%d\r\n", *pid, *data_length, head, *queue_len);

		return 1; // Success
	}
	return 0;
}

/**
 * Removes the first queued message if it matches with the given PID
 * Returns 1 if a message was available, 0 if the queue is empty or PID not found in head.
 */
uint8_t DmxRdm_Module_RemoveQueuedMessage(uint16_t idx, uint16_t sub_idx, uint16_t pid)
{
	if(idx == 0)
	{
		if (rdm_queue_size == 0) {
			return 0; // Nu messages in queue
		}

		uint8_t head = rdm_queue_head;
		if(rdm_queue[head].pid == pid)
		{
			rdm_queue_head = (head + 1) % MAX_QUEUED_MESSAGES;
			rdm_queue_size--;
			OS_TRACE_INFO("Queue PID remove[%d]: 0x%04X (%d)\r\n", idx, pid, head);
			return 1; // Success
		}
	}
	return 0;
}

uint8_t DmxRdm_Module_GetQueuedLength(uint16_t idx, uint16_t sub_idx)
{
	return rdm_queue_size;
}

/**
 * Retrieves the next status message from the queue.
 * Returns 1 if a message was available, 0 if the queue is empty.
 */
uint8_t DmxRdm_Module_GetStatusMessage(uint16_t idx, uint8_t req_type, uint16_t *sub_idx, uint8_t* status_type, uint16_t* status_id, uint8_t* data, uint8_t* data_length)
{
    /*if(idx == 0)
    {
    	uint8_t status_value = 0, rstatus_type = E120_STATUS_ADVISORY;
    	uint16_t rstatus_value = 0;

    	int8_t res = DmxRdm_Module_GetStatus(idx, &status_value, &rstatus_type, &rstatus_value);
		if(res < 0)
			return 0;

    	uint8_t last_state = rdm_modules[idx].last_state;
		if(status_value != last_state || (rdm_modules[idx].flags & DMX_RDM_REPORT_LASTSTATE) || req_type == E120_STATUS_GET_LAST_MESSAGE)
		{
			if(req_type > E120_STATUS_GET_LAST_MESSAGE && rstatus_type < req_type)		// Advisory return all, if Error is requested only return errors
				return 0;

			if(rstatus_value > 0)
				*status_id = DMX_RDM_STATUS_MAN_SPECIFIC + rstatus_value;
			else
				*status_id = 0x50;//STS_READY;
			*status_type = rstatus_type;
			rdm_modules[idx].flags &= ~DMX_RDM_REPORT_LASTSTATE;

			//OS_TRACE_ERROR("Set status: %d != %d (type: %d, id: 0x%02x)\r\n", status_value, last_state, *status_type, *status_id);
		}
		else
			return 0;

		*sub_idx = 0;
		memset(data, 0, 4);
		*data_length = 4;

		rdm_modules[idx].last_state = status_value;

		return 1; // Success 1 = last, > 1 is more to come
    }*/
    return 0;			// Nothing found
}

const char* DmxRdm_Module_GetStatusDescription(uint16_t status_id)
{
	
	return "Unknown";
}

/*void DmxRdm_Module_ClearAllStatusMessages()
{

}*/


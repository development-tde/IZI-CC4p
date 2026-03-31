/*
 * dmx_rdm_module.h
 *
 *  Created on: 27 jan. 2025
 *      Author: Milo
 */

#ifndef DMX_RDM_MODULE_H_
#define DMX_RDM_MODULE_H_

// ----- Defines -----
#define DMX_RDM_MODULE_MAX		128

#define DMX_RDM_FLAG_MUTE		0x01
#define DMX_RDM_FLAG_IDENTIFY	0x02

#define DMX_RDM_BC_INDEX		-32767

#define DMX_RDM_TYPE_FIXTURE	0
#define DMX_RDM_TYPE_IZIACCESS	1
#define DMX_RDM_TYPE_CAN		4

#define MONTYPE_CONTACTS	1
#define MONTYPE_STATUS		2

#define MONTYPE_REFID_SUPPLY_VOLT	1			// Supply voltage
#define MONTYPE_REFID_LED_TEMP		3			// LED emitter temperature
#define MONTYPE_REFID_INTERN_TEMP	4			// Intern temperature
#define MONTYPE_REFID_NTC1_TEMP		5			// NTC1 temperature
#define MONTYPE_REFID_NTC2_TEMP		6			// NTC2 temperature
#define MONTYPE_REFID_POWER20		20			// Power in 1/20W
#define MONTYPE_REFID_STATUS		21			// Status of device
#define MONTYPE_REFID_POWER6		22			// Power in 1/6W
#define MONTYPE_REFID_POWER3		24			// Power in 1/3W
#define MONTYPE_REFID_ACT_CONTACTS	25			// Contacts active (bitmap)
#define MONTYPE_REFID_INPUTS		28			// Inputs active (bitmap)
#define MONTYPE_REFID_UPTIME		32			// Up time, operation time
#define MONTYPE_REFID_ONTIME		35			// On time / burning time

#define CFGTYPE_REFID_CURRENT_1050	1			// Current set (max 1050)
#define CFGTYPE_REFID_MAXOUTPUT		26			// Max output (0 to 100%)
#define CFGTYPE_REFID_TTW_CCT_COL	32			// TTW CCT Color Range
#define CFGTYPE_REFID_FILTER_MODE	34			// Filter mode
#define CFGTYPE_REFID_DIM_CURVE		35			// Dim curve select
#define CFGTYPE_REFID_TW_CCT_MIN	45			// TW CCT minimum
#define CFGTYPE_REFID_TW_CCT_MAX	46			// TW CCT maximum
#define CFGTYPE_REFID_WD_CCT_MIN	47			// WD CCT minimum
#define CFGTYPE_REFID_WD_CCT_MAX	48			// WD CCT maximum

#define MONTYPE_REFID_SUPPLY_VOLTB	49			// Supply voltage B (not defined yet in TDE file!)

#define CFGTYPE_REFID_DMXFAIL_PUP				  0x0080
#define CFGTYPE_REFID_DMXFAIL_OPERATION			  0x0081

typedef struct rdm_module_s {
	uint8_t		flags;
	uint8_t		type;
	uint16_t	idx;
	uint16_t    device_model;
	uint16_t 	product_category;
	uint32_t	serial;
	uint32_t	sensor_flags;
	uint8_t		personalities;
	uint8_t		sub_idx;				// Sub device number
	uint8_t		sub_devices;
	uint8_t		sensors;
}rdm_module_t;

#define MAX_STATUS_MESSAGES 5  // Adjust as needed
#define MAX_STATUS_DATA_SIZE 4 // Optional extra data size

typedef struct rdm_module_queue_s {
	uint16_t pid;
	uint8_t length;             // Length of data to fake for input (for now only 0 or 1)
	uint8_t data;				// Data for faked input
}rdm_module_queue_t;

#define MAX_QUEUED_MESSAGES 8  // Adjust as needed

// Proto
void DmxRdm_Module_Init();
void DmxRdm_Module_Update();

int16_t DmxRdm_Module_SearchSerial(uint32_t serial, uint8_t *sub_amount);
rdm_module_t *DmxRdm_Module_GetIdx(int16_t idx);
void DmxRdm_Module_Mute(bool mute, int16_t idx);
int16_t DmxRdm_Module_GetUnmuted();
bool DmxRdm_Module_GetDeviceID(int16_t idx, DEVICEID dev_id);
void DmxRdm_Module_Delayed();
void DmxRdm_Module_Timer();

bool DmxRdm_Module_GetDeviceInfo(int16_t idx, uint16_t sub_idx, DEVICEINFO *devInfo);
uint8_t DmxRdm_Module_GetModelName(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len);
uint8_t DmxRdm_Module_GetLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len);
bool DmxRdm_Module_SetLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t len);
bool DmxRdm_Module_SetIdentify(int16_t idx, uint16_t sub_idx, bool on);
bool DmxRdm_Module_GetIdentify(int16_t idx, uint16_t sub_idx);
bool DmxRdm_Module_SetReset(int16_t idx, uint16_t sub_idx, uint8_t type);
bool DmxRdm_Module_SetAddress(int16_t idx, uint16_t sub_idx, uint16_t address);
uint16_t DmxRdm_Module_GetAddress(int16_t idx, uint16_t sub_idx);
uint8_t DmxRdm_Module_GetVersionLabel(int16_t idx, uint16_t sub_idx, char* name, uint8_t max_len, bool boot);
uint8_t DmxRdm_Module_GetPersonalityDescription(int16_t idx, uint16_t sub_idx, uint8_t pers, char* name, uint8_t max_len, uint8_t *channels);
bool DmxRdm_Module_SetPersonality(int16_t idx, uint16_t sub_idx, uint8_t pers);
int8_t DmxRdm_Module_GetPersonality(int16_t idx, uint16_t sub_idx, uint8_t *max_pers);
bool DmxRdm_Module_SetLampState(int16_t idx, uint16_t sub_idx, uint8_t state);
uint8_t DmxRdm_Module_GetLampState(int16_t idx, uint16_t sub_idx);
uint32_t DmxRdm_Module_GetDeviceHours(int16_t idx, uint16_t sub_idx);
uint32_t DmxRdm_Module_GetBurnHours(int16_t idx, uint16_t sub_idx);
uint8_t DmxRdm_Module_GetSupportedParameters(int16_t idx, uint16_t sub_idx, uint16_t *parameters, uint8_t max_parameters);
PARAMETER_DESCRIPTION *DmxRdm_Module_GetDescriptionParameters(int16_t idx, uint16_t sub_idx, uint16_t pid);
SENSORDEF* DmxRdm_Module_GetSensorDef(int16_t idx, uint16_t sub_idx, uint8_t sensor_idx);
uint32_t DmxRdm_Module_GetSensorValue(int16_t idx, uint16_t sub_idx, uint8_t sensor_idx, uint16_t *data);
bool DmxRdm_Module_SetTdeSpecific(int16_t idx, uint16_t sub_idx, uint16_t pid, uint16_t data);
int16_t DmxRdm_Module_GetTdeSpecific(int16_t idx, uint16_t sub_idx, uint16_t pid, uint16_t *data);
uint8_t DmxRdm_Module_AddQueuedMessage(int16_t idx, uint16_t sub_idx, uint16_t pid, const uint8_t* data, uint8_t data_length);
uint8_t DmxRdm_Module_GetQueuedMessage(uint16_t idx, uint16_t sub_idx, uint16_t* pid, uint8_t* data, uint8_t* data_length, uint8_t *queue_len);	
uint8_t DmxRdm_Module_RemoveQueuedMessage(uint16_t idx, uint16_t sub_idx, uint16_t pid);
uint8_t DmxRdm_Module_GetQueuedLength(uint16_t idx, uint16_t sub_idx);
uint8_t DmxRdm_Module_GetStatusMessage(uint16_t idx, uint8_t req_type, uint16_t *sub_idx, uint8_t* status_type, uint16_t* status_id, uint8_t* data, uint8_t* data_length);
const char* DmxRdm_Module_GetStatusDescription(uint16_t status_id);

#endif /* DMX_RDM_MODULE_H_ */

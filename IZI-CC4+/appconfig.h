/*
 * appconfig.h
 *
 *  Created on: 15 jun. 2020
 *      Author: Milo
 */

#ifndef SOURCE_APPCONFIG_H_
#define SOURCE_APPCONFIG_H_

#include "izi_module.h"

#define CONFIG_VERSION	7
#define CALIB_VERSION	1
#define LOG_VERSION		1
#define LOGE_VERSION	1

#define NAME_SIZE		32
#define PROJECT_SIZE	16

#define APPSETTING_SECTOR_SIZE		512

#define APPCFG_DFLT_INCOMPLETE		0xA5
#define APPCFG_DFLT_OK				0xCD

// Position of config (gaps are not supported right now)
#define APPCFG_CURRENT				0
#define APPCFG_FILTERMODE			1
#define APPCFG_NTC1					2		
#define APPCFG_NTC2					3
#define APPCFG_CHAN_SWAP			4
#define APPCFG_CONTACT_MAP			5
#define APPCFG_CURVE				6

typedef struct appconfig_s {
	// General
	uint32_t crc;
	uint32_t ref;				// time of file (used in code as first after crc, so do not move)
	uint32_t version;
	uint32_t serial;
	char name[NAME_SIZE];
	uint32_t pan_id;
	uint8_t pl_frequency;
	uint8_t pl_rate;
	uint8_t dbg_on;
	uint8_t short_id;
	uint16_t location_id;
	uint16_t channel;
	uint8_t mode;
	uint8_t dmxfail;
	uint8_t offset;
	uint8_t pl_txlevel;
	uint8_t config[IZI_MAX_CONFIG];		
	char dummy[APPSETTING_SECTOR_SIZE-100];				// Filler (page), check AppConfig_Set if larger than 128 (is page size)
	uint8_t res;
	uint8_t default_ok;		// Check if default was incomplete because emitter was not known yet
	uint8_t variant;		// Last variant detected at PUP
	uint8_t typeid;			// Last type detected at PUP
} appconfig_t;

#define APPCALIB_SECTOR_SIZE		128

typedef struct appcalib_s {
	// General
	uint32_t crc;
	uint32_t ref;										// time of file (used in code as first after crc, so do not move)
	uint32_t version;
	uint16_t pwm_offset[4];
	uint16_t vled_max_mv[4];
	char dummy[APPCALIB_SECTOR_SIZE-28];				// Filler (page), check AppCalib_Set if larger than 128 (is page size)
} appcalib_t;

#define APPLOG_SECTOR_SIZE		128

typedef struct applog_s {
	// General
	uint32_t crc;
	uint32_t ref;										// time of file (used in code as first after crc, so do not move)
	uint32_t version;
	uint32_t operating_sec;
	uint32_t active_sec;
	uint16_t intern_temp_max;
	uint16_t ntc_temp_max;
	uint16_t ntc2_temp_max;
	uint16_t supply_min;
	uint16_t change_count;
	uint8_t min_output;
	uint32_t active_ch_sec[4];							// Time time the channel was on (on > 0%)
	char dummy[APPLOG_SECTOR_SIZE-53];					// Filler (page), check AppCalib_Set if larger than 64 (is page size)
} applog_t;

extern appcalib_t *appCalibData;
extern applog_t *appLogData;

extern appconfig_t *appConfig;

#define APPMODULES_MAX		32
typedef struct appmodule_s {
	izi_module_base_t modules[APPMODULES_MAX];
}appmodule_t;

// Proto
void AppConfig_Init();
void AppConfig_Default();
void AppConfig_Print();
appconfig_t *AppConfig_Get();
appconfig_t *AppConfig_GetPtr();
bool AppConfig_Set();
void AppConfig_CheckConfig();
void AppCalib_Default();
appcalib_t *AppCalib_Get();
bool AppCalib_Set();
applog_t *AppLog_Get();
void AppLog_Default(bool all);
bool AppLog_Set();

#endif /* SOURCE_APPCONFIG_H_ */

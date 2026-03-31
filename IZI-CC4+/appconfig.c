/*
 * appconfig.c
 *
 *  Created on: 15 jun. 2020
 *      Author: Milo
 */

#include "includes.h"
#include "appconfig.h"
#include "crc.h"
#include "smart_eeprom.h"
#include "iziplus_module_def.h"
#include "11LC160.h"

#define APPCFG_GEN			1
#define APPCFG_NETWORK		8

uint32_t AppConfig_Changes = 0;

//const uint8_t  __attribute__ ((section (".appsettingsection"))) appsettings;
appconfig_t appCfg;
appconfig_t *appConfig;

appcalib_t appCalib;
appcalib_t *appCalibData;

applog_t __attribute__ ((section (".energy_ram"))) appLog;
applog_t *appLogData;

void AppConfig_Init()
{
	appconfig_t *cfg = AppConfig_Get();
	appConfig = cfg;
	
	uint16_t crc = Crc16((uint8_t *)&cfg->ref, (uint16_t)(sizeof(appconfig_t) - sizeof(cfg->crc)), 0xFFFF);
	if(cfg->crc != crc)
	{
		GEN_TRACE_INFO("App settings crc fail\r\n");
		AppConfig_Default();
	}
	else
	{
		GEN_TRACE_INFO("App settings OK\r\n");
		if(cfg->version < CONFIG_VERSION)
		{
			if(cfg->version < 2)
			{
				cfg->location_id = 0;
			}
			if(cfg->version < 8)
			{
				cfg->config[APPCFG_CURVE] = 2;
			}
			cfg->version = CONFIG_VERSION;
			AppConfig_Set();
		}
	}
	
	uint8_t offset = 0;
	bool store = false;
	for(int i = 0; i < CONFIG_AMOUNT; i++)
	{
		if(iziplus_fixdef.configs[i].size == 1)			// 8-bit
		{
			uint8_t data = cfg->config[i + offset];
			if(data < iziplus_fixdef.configs[i].min || data > iziplus_fixdef.configs[i].max)
			{
				cfg->config[i + offset] = iziplus_fixdef.configs[i].dflt;
				store = true;
			}
			// TODO 16 bit
		}
		offset += (iziplus_fixdef.configs[i].size - 1);
	}
	if(store)
		AppConfig_Set();

	applog_t *log = AppLog_Get();
	appLogData = log;
	
	crc = Crc16((uint8_t *)&log->ref, (uint16_t)(sizeof(applog_t) - sizeof(log->crc)), 0xFFFF);
	if(log->crc != crc)
	{
		GEN_TRACE_INFO("App log crc fail\r\n");
		AppLog_Default(true);
	}
	else
	{
		GEN_TRACE_INFO("App log OK\r\n");
		if(log->version < LOG_VERSION)
		{
			log = AppLog_Get();
			log->version = LOG_VERSION;
			AppLog_Set();
		}
	}
	
	appcalib_t *calib = AppCalib_Get();
	appCalibData = calib;
	
	crc = Crc16((uint8_t *)&calib->ref, (uint16_t)(sizeof(appcalib_t) - sizeof(calib->crc)), 0xFFFF);
	if(calib->crc != crc)
	{
		GEN_TRACE_INFO("App calibration crc fail\r\n");
		AppCalib_Default();
	}
	else
	{
		GEN_TRACE_INFO("App calibration OK\r\n");
		if(calib->version < CALIB_VERSION)
		{
			calib = AppCalib_Get();
			calib->version = CALIB_VERSION;
			AppCalib_Set();
		}
	}
	// Do reading of ext mem later
}

void AppCalib_Default()
{
	appcalib_t *calib = &appCalib;
	
	// General
	calib->ref = 0;
	calib->version = CALIB_VERSION;
	
	calib->vled1_max_mv = calib->vled2_max_mv = 0;
	calib->pwm1_offset = calib->pwm2_offset = 100;
	
	uint16_t crc = Crc16((uint8_t *)&calib->ref, (uint16_t)(sizeof(appconfig_t) - sizeof(calib->crc)), 0xFFFF);
	calib->crc = crc;
	
	if(Eeprom_Write(EEPROM_CALIBRATE, (uint8_t *)calib, APPCALIB_SECTOR_SIZE))
	{
		GEN_TRACE_INFO("App calib default OK\r\n");
		AppCalib_Get();
	}
	else
	GEN_TRACE_INFO("App calib default error\r\n");
}

void AppConfig_Default()
{
	appconfig_t *cfg = &appCfg;
	
	// General
	cfg->ref = 0;
	cfg->version = CONFIG_VERSION;
	
	/*if(variant == 1)
		strcpy(cfg->name, "MiniMoodspot+ WD");
	else
		strcpy(cfg->name, "MiniMoodspot+");*/
	memset(cfg->name, 0, sizeof(cfg->name));
	
	cfg->pl_frequency = 0x50;
	cfg->pl_rate = 0x03;
	cfg->pan_id = 0xFFFFFFFF;
	cfg->dbg_on = 0;
	cfg->short_id = 0xFF;
	cfg->channel = 0;
	//cfg->mode = 2;			
#if DEBUG
	cfg->dmxfail = 0x00;
#else
	cfg->dmxfail = 0x12;
#endif
	cfg->offset = 0;
	cfg->location_id = 0;
	cfg->pl_txlevel = 0xF0;
	cfg->variant = 0;
	cfg->typeid = 0xFF;
	
	cfg->typeid = 0;
	cfg->mode = iziplus_fixdef.default_mode;
		
	uint8_t offset = 0;
	memset(cfg->config, 0, 24);
	for(int i = 0; i < CONFIG_AMOUNT; i++)
	{
		memcpy(&cfg->config[i + offset], &iziplus_fixdef.configs[i].dflt, iziplus_fixdef.configs[i].size);
		offset += (iziplus_fixdef.configs[i].size - 1);
	}
	cfg->default_ok = APPCFG_DFLT_OK;
		
	uint16_t crc = Crc16((uint8_t *)&cfg->ref, (uint16_t)(sizeof(appconfig_t) - sizeof(cfg->crc)), 0xFFFF);
	cfg->crc = crc;
	
	if(Eeprom_Write(EEPROM_APPCFG, (uint8_t *)cfg, APPSETTING_SECTOR_SIZE))
	{
		GEN_TRACE_INFO("App default OK\r\n");
		AppConfig_Get();
	}
	else
		GEN_TRACE_INFO("App default error\r\n");
}

appconfig_t *AppConfig_Get()
{
	Eeprom_Read(EEPROM_APPCFG, (uint8_t *)&appCfg, APPSETTING_SECTOR_SIZE);
	return &appCfg;
}

appconfig_t *AppConfig_GetPtr()
{
	return &appCfg;
}


bool AppConfig_Set()
{
	TickType_t ticks = xTaskGetTickCount();

	uint16_t crc = Crc16((uint8_t *)&appCfg.ref, (uint16_t)(sizeof(appconfig_t) - sizeof(appCfg.crc)), 0xFFFF);
	appCfg.crc = crc;
	
	//GEN_TRACE_INFO("**** Hai hai\r\n");
	
	if(Eeprom_Write(EEPROM_APPCFG, (uint8_t *)&appCfg, APPSETTING_SECTOR_SIZE))
	{
		IziPlus_Module_Update();
		StepDown_Module_Update();
		
		GEN_TRACE_INFO("App write ram settings %d ms\r\n", xTaskGetTickCount() - ticks);
		return true;
	}
	else
		GEN_TRACE_INFO("App write error\r\n");
	return false;
}

void AppConfig_Print()
{
	GEN_TRACE_INFO("General\r\n");
	GEN_TRACE_INFO("Serial    : %08x\r\n", PRODUCTION_SERIAL);
	GEN_TRACE_INFO("Batch     : %s\r\n", PRODUCTION_BATCH);
	GEN_TRACE_INFO("Plant     : %c\r\n", PRODUCTION_PLANT);
	GEN_TRACE_INFO("Version   : %d\r\n", appConfig->version);
	GEN_TRACE_INFO("Name      : %s\r\n", appConfig->name);
	GEN_TRACE_INFO("IO\r\n");
	GEN_TRACE_INFO("Maximum Out: %d\r\n", appConfig->config[0]);
	GEN_TRACE_INFO("TTW CCT Col: %d\r\n", appConfig->config[1]);
	GEN_TRACE_INFO("Filter mode: %d\r\n", appConfig->config[2]);
	GEN_TRACE_INFO("Free config: %d\r\n", appConfig->config[3]);
	GEN_TRACE_INFO("IZI\r\nPAN ID:      %x\r\n", appConfig->pan_id);
	GEN_TRACE_INFO("Channel:     %d\r\n", appConfig->channel);
	GEN_TRACE_INFO("Emitter\r\nTypeID:      %x\r\n", appConfig->typeid);
	GEN_TRACE_INFO("Variant:     %d\r\n", appConfig->variant);
}

appcalib_t *AppCalib_Get()
{
	Eeprom_Read(EEPROM_CALIBRATE, (uint8_t *)&appCalib, APPCALIB_SECTOR_SIZE);
	return &appCalib;
}

/************************************************************************/
/* Brief: Calculate crc and write into smart eeprom                     */
/************************************************************************/
bool AppCalib_Set()
{
	TickType_t ticks = xTaskGetTickCount();

	uint16_t crc = Crc16((uint8_t *)&appCalib.ref, (uint16_t)(sizeof(appcalib_t) - sizeof(appCalib.crc)), 0xFFFF);
	appCalib.crc = crc;
	
	if(Eeprom_Write(EEPROM_CALIBRATE, (uint8_t *)&appCalib, APPCALIB_SECTOR_SIZE))
	{
		GEN_TRACE_INFO("App calib write ram settings %d ms\r\n", xTaskGetTickCount() - ticks);
		return true;
	}
	else
	GEN_TRACE_INFO("App calib write error\r\n");
	return false;
}

void AppLog_Default(bool all)
{
	applog_t *log = &appLog;
	
	// General
	log->ref = 0;
	log->version = LOG_VERSION;
	
	if(all)
		log->operating_sec = log->active_sec = 0;
	log->intern_temp_max = log->ntc_temp_max = 0;
	log->supply_min = 55000;		// 55V
	log->min_output = 100;			// 100%
	log->change_count = 0;
	memset(log->active_ch_sec, 0, 4);
	
	uint16_t crc = Crc16((uint8_t *)&log->ref, (uint16_t)(sizeof(applog_t) - sizeof(log->crc)), 0xFFFF);
	log->crc = crc;
	
	if(Eeprom_Write(EEPROM_LOG, (uint8_t *)log, APPLOG_SECTOR_SIZE))
	{
		GEN_TRACE_INFO("App log default OK\r\n");
		AppLog_Get();
		
		IziPlus_Module_Update();
		StepDown_Module_Update();
	}
	else
		GEN_TRACE_INFO("App log default error\r\n");
}

void AppConfig_CheckConfig()
{
	appconfig_t *cfg = &appCfg;
	uint8_t changes = 0;
	uint16_t offset = 0;
	
	for(int i = 0; i < CONFIG_AMOUNT; i++)
	{
		uint16_t data = iziplus_fixdef.configs[i].size == 2 ? (cfg->config[i + offset] + (cfg->config[i+1+offset] << 8)) : cfg->config[i+offset];
		if(data < iziplus_fixdef.configs[i].min || data > iziplus_fixdef.configs[i].max)
		{
			memcpy(&cfg->config[i + offset], &iziplus_fixdef.configs[i].dflt, iziplus_fixdef.configs[i].size);
			changes++;
		}
		offset += (iziplus_fixdef.configs[i].size - 1);
	}
	
	if(changes)
	AppConfig_Set();
}

applog_t *AppLog_Get()
{
	Eeprom_Read(EEPROM_LOG, (uint8_t *)&appLog, APPLOG_SECTOR_SIZE);
	return &appLog;
}

/************************************************************************/
/* Brief: Calculate crc and write into smart eeprom                     */
/************************************************************************/
bool AppLog_Set()
{
	TickType_t ticks = xTaskGetTickCount();

	appLog.change_count = 0;
	uint16_t crc = Crc16((uint8_t *)&appLog.ref, (uint16_t)(sizeof(applog_t) - sizeof(appLog.crc)), 0xFFFF);
	appLog.crc = crc;
	
	if(Eeprom_Write(EEPROM_LOG, (uint8_t *)&appLog, APPLOG_SECTOR_SIZE))
	{
		GEN_TRACE_INFO("Log write ram ok %d ms\r\n", xTaskGetTickCount() - ticks);
		return true;
	}
	else
		GEN_TRACE_INFO("Log write error\r\n");
	
	return false;
}


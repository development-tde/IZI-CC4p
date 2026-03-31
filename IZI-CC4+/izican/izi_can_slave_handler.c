/*
 * izican_slave_handler.c
 *
 *  Created on: 9 jul. 2020
 *      Author: Milo
 */



#include "includes.h"
#include "izi_can_slave_handler.h"
#include "izi_can_slave.h"
#include "izi_can_frame.h"
#include "izi_can_driver.h"
#include "rtctime.h"
#include "appconfig.h"
//#include "fsl_flashiap.h"
#include "firmware_info.h"
#include "crc.h"
#include "logger_data.h"
#if IZIPLUS_DRIVER > 0
#include "izilink/iziplus_driver.h"
#endif
#if IZI_DRIVER
#include "izi_driver.h"
#endif
#include "version.h"
//#include "fsl_rng.h"
#include "logger_data.h"
#include "misc/state.h"

// Defines
#define FLASH_APP_PROG_SIZE		256

// Globals
uint8_t can_slave_id;
uint16_t can_slave_state;

uint32_t can_prog_address = 0, can_prog_offset = 0;
uint32_t can_firmware_size = 0;

//uint8_t can_prog_ram_bfr[RAM_APP_PROG_SIZE];
fw_mem_location_t can_prog_fw_mem = fw_mem_invalid;

//const uint8_t  __attribute__ ((section (".mirrorsection"))) mirrorflash;
const uint8_t  __attribute__ ((section (".bootloadersection"))) bootloaderflash;

// Proto
extern boot_t boot_data;
extern void IziPlus_SyncTimers();

void izican_slave_erase_address()
{
	if(appConfig->can_slave_id < ADDRESS_WILDCARD)
	{
		appconfig_t *cfgram = AppConfig_Get();
		cfgram->can_slave_id = ADDRESS_WILDCARD;		// Erase slave id, until renewed by assign command
		AppConfig_Set();
		
		IZI_CAN_TRACE_INFO("Erased slave address %d %d\r\n", appConfig->can_slave_id, cfgram->can_slave_id);
	}
	can_slave_id = ADDRESS_WILDCARD;
	izican_set_slave_address(ADDRESS_WILDCARD);
	can_slave_state = IZICAN_STATE_NOT_ASSIGNED;
}

bool izican_slave_assign_address(uint32_t network_id, uint8_t address, uint8_t timing_offset)
{
	appconfig_t *cfgram = AppConfig_Get();
	cfgram->can_slave_id = address;					// Set new slave address
	cfgram->can_network_id = network_id;			// Store network (serial of master) as well
	cfgram->timing_offset = timing_offset;
	can_slave_id = address;
	bool res = AppConfig_Set();
	izican_set_slave_address(address);
	if(can_slave_state == IZICAN_STATE_NOT_ASSIGNED)
		can_slave_state = IZICAN_STATE_OK;

	IZI_CAN_TRACE_INFO("Set slave address %d (network %d): %d\r\n", appConfig->can_slave_id, appConfig->can_network_id, res);
	return res;
}

extern uint16_t iziplus_local_identify_time;
uint32_t last_errors = 0;
uint32_t last_warnings = 0;
uint32_t last_start_info = 0;
uint16_t last_can_slave_state = 0;

int16_t izican_slave_update_state(bool report, bool force)
{
	if(xTaskGetTickCount() < 5000)
		;											// Don't update state the first 5 sec to report reset cause
	else if(State_GetErrors() > 0)
		can_slave_state = IZICAN_STATE_ERROR;
	else if(State_GetWarnings() > 0)
		can_slave_state = IZICAN_STATE_WARNING;
	else if(can_slave_id == ADDRESS_WILDCARD)
		can_slave_state = IZICAN_STATE_NOT_ASSIGNED;
	else if(iziplus_local_identify_time > 0)
		can_slave_state = IZICAN_STATE_IDENTIFY;
	else
		can_slave_state = IZICAN_STATE_OK;
	
	if(report)
	{
		uint32_t errors = State_GetErrors();
		uint32_t warnings = State_GetWarnings();
		uint32_t start_info = State_GetStartInfo();
		//uint32_t run_info = State_GetRunInfo();

		char *state_text = "";
		if(errors > 0)
			state_text = State_GetHighestErrorText();
		else if(warnings > 0)
			state_text = State_GetHighestWarningText();
		else if(start_info > 0)
			state_text = State_GetHighestStartInfoText();
		
		if(force || errors != last_errors || warnings != last_warnings || start_info != last_start_info || can_slave_state != last_can_slave_state)
			izican_slave_base_state(can_slave_state, errors, warnings, start_info, state_text);
		
		last_errors = errors;
		last_warnings = warnings;
		last_start_info = start_info;
	}
	
	last_can_slave_state = can_slave_state;
	return can_slave_state;
}

uint8_t last_monitor_values[32];
bool izican_slave_update_monitor(uint8_t *values, uint8_t length, bool force)
{
	uint8_t len = length > sizeof(last_monitor_values) ? sizeof(last_monitor_values) : length;
	
	if(force || (memcmp(values, last_monitor_values, len) != 0))
	{
		izican_slave_monitor_report(values, len);
		memcpy(last_monitor_values, values, len);
		return true;
	}
	return false;
}

static TickType_t can_master_hb_ticks = 0x80000000;

bool izican_master_present(void)
{
	return xTaskGetTickCount() - can_master_hb_ticks < 2500;
}

void izican_slave_notification_handler(izi_can_extid_u id, izican_notification_data_t *notification, uint16_t length)
{
	if(id.bit.command == IZI_CAN_CMD_HEARTBEAT && id.bit.master_id == 0)
	{
		if((notification->data.master_hb.datetime % 4) == 0)
			State_TimerSync();
		IziPlus_SyncTimers(notification->data.master_hb.datetime);
		
		if(notification->data.master_hb.serial_master != appConfig->can_network_id)
		{
			izican_slave_erase_address();
		}
		else 
			can_slave_state = izican_slave_update_state(false, false);

		if(abs(RtcTime_GetSeconds() - notification->data.master_hb.datetime) > 2)		// Correction when more than 2 sec difference
			RtcTime_SetTime(notification->data.master_hb.datetime);

		can_master_hb_ticks = xTaskGetTickCount();
	}
	else if(id.bit.command == IZI_CAN_CMD_DBG && id.bit.master_id == 0)
	{
		for(int i = 0; i < length; i++)
			Debug_Recv(notification->data.dbg_report.txt[i]);
		//IZI_CAN_TRACE_INFO("Dbg msg: %s", notification->data.dbg_report.txt);
	}
}

static TickType_t ticks2;
void izican_slave_request_handler(izi_can_extid_u id, izican_request_data_t *req, uint16_t length)
{
	if(id.bit.command == IZI_CAN_CMD_DISCOVER)
	{
		if(length < sizeof(izican_req_discover_t))
			return;
			
		TickType_t ticks = xTaskGetTickCount();
		izican_req_discover_t *izi_can_data = &req->data.discover;
		if(izi_can_data->serial_master != appConfig->can_network_id || izi_can_data->options & IZI_CAN_DISCOVER_OPTION_CLEAN)
			izican_slave_erase_address();

		izican_rsp_discover_t rsp_discover;
		rsp_discover.serial_slave = PRODUCTION_SERIAL;
		rsp_discover.cfg_version = (uint16_t)appConfig->version;
		rsp_discover.app_version = firmare_info.version.all;
		version_u *boot_version = (version_u *)BOOT_VERSION_ADDRESS;
		rsp_discover.boot_version = boot_version->all;
		rsp_discover.session_id = izi_can_data->session_id;
		rsp_discover.pl_frequency = appConfig->pl_frequency;
		rsp_discover.pl_rate = appConfig->pl_rate;
		rsp_discover.timing_offset = appConfig->timing_offset;
		rsp_discover.cfg_len = APPSETTING_MAX_PC;				
		memcpy(rsp_discover.config, appConfig->pc_config, APPSETTING_MAX_PC);		// Send the first 8 cfg bytes
		rsp_discover.options = 0;
		rsp_discover.hw_id = get_hw_id();
		rsp_discover.dev_id = get_device_id();
#if IZIPLUS_DRIVER > 0
		rsp_discover.module_cnt = IziPlus_ModuleAmount();
#elif IZI_DRIVER
		rsp_discover.module_cnt = izi_get_module_amount();
#else
		rsp_discover.module_cnt = 0;
#endif
		strncpy((char *)rsp_discover.name, appConfig->name, sizeof(rsp_discover.name));

		uint16_t random_wait = /*xTaskGetTickCount()*/ rand_sync_read32(&RAND_0) % izi_can_data->random_rsp; 
		IZI_CAN_TRACE_INFO("IZI_CAN_CMD_DISCOVER start (delay: %d)\r\n", random_wait);
		if(random_wait > 0)
			vTaskDelay(random_wait);
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_DISCOVER, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_discover, sizeof(izican_rsp_discover_t), id.bit.seq_id);
		
		IZI_CAN_TRACE_INFO("IZI_CAN_CMD_DISCOVER end (interval: %d)\r\n", xTaskGetTickCount() - ticks);
	}
	else if(id.bit.command == IZI_CAN_CMD_ASSIGN)
	{
		if(length < sizeof(izican_req_assign_t))
			return;
		
		izican_req_assign_t *izi_can_data = &req->data.assign;
		if(izi_can_data->serial_slave == PRODUCTION_SERIAL)			// Addressed to us?
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_ASSIGN start (address: %d)\r\n", izi_can_data->address);
			
			bool res = izican_slave_assign_address(izi_can_data->serial_master, izi_can_data->address, izi_can_data->timing_offset);

			izican_rsp_assign_t rsp_assign;
			rsp_assign.serial_slave = PRODUCTION_SERIAL;
			rsp_assign.result = res ? 0 : -1;
			izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_ASSIGN, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_assign, sizeof(izican_rsp_assign_t), id.bit.seq_id);
		}
	}
	else if(id.bit.command == IZI_CAN_CMD_TIMING)
	{
		if(length < sizeof(izican_req_timing_t))
			return;
			
		izican_req_timing_t *izi_can_data = &req->data.timing;
		appconfig_t *cfgram = AppConfig_Get();
		cfgram->timing_offset = izi_can_data->timing_offset;
		cfgram->master_idx = izi_can_data->master_idx;
		cfgram->master_total = izi_can_data->master_total;
		bool res = AppConfig_Set();

		izican_rsp_timing_t rsp_timing;
		rsp_timing.result = res ? 0 : -1;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_TIMING, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_timing, sizeof(izican_rsp_timing_t), id.bit.seq_id);
		IZI_CAN_TRACE_INFO("IZI_CAN_CMD_TIMING start (offset: %d)\r\n", izi_can_data->timing_offset);
	}
	else if(id.bit.command == IZI_CAN_CMD_POSITION)
	{
		if(length < sizeof(izican_req_position_t))
			return;
		
		izican_req_position_t *izi_can_data = &req->data.position;
		if(izi_can_data->activate != 0xFF)		// New state should be set?
		{
#ifdef PCB_REV6
			gpio_set_pin_level(CAN_CHK, izi_can_data->activate == 0);
#else
			gpio_set_pin_level(CAN_CHK, get_hw_id() >= 4 ? izi_can_data->activate > 0 : izi_can_data->activate == 0);
#endif			
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_POSITION Set position %d (a: %d)\r\n", izi_can_data->activate, id.bit.address);
		}

		if(id.bit.address != ADDRESS_BROADCAST)
		{
			izican_rsp_position_t izican_rsp_position;
#ifdef PCB_REV6
			izican_rsp_position.state = gpio_get_pin_level(CAN_SNS);	
#else			
			izican_rsp_position.state = get_hw_id() >= 3 ? !gpio_get_pin_level(CAN_SNS) : gpio_get_pin_level(CAN_SNS);	
#endif			
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_POSITION Get position %d\r\n", izican_rsp_position.state);
			izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_POSITION, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&izican_rsp_position, sizeof(izican_rsp_position_t), id.bit.seq_id);
		}
	}
	/*else if(id.bit.devtype_group != SLAVE_DEVTYPE_GROUP)
	{
		// This is not good
	}*/
	else if(id.bit.command == IZI_CAN_CMD_BASE_SETCONFIG)
	{
		izican_rsp_base_setconfig_t rsp_base_setconfig;
		if(length >= IZI_CAN_CMD_BASE_MINCONFIG)
		{			
			izican_req_base_setconfig_t *izi_can_data = &req->data.base_setconfig;
		
			appconfig_t *cfgram = AppConfig_Get();
			strncpy(cfgram->name, izi_can_data->name, IZI_CAN_CMD_BASE_NAME_SIZE);
			cfgram->name[30] = 0;
			cfgram->location_id = izi_can_data->location_id;
			if(length >= IZI_CAN_CMD_BASE_MINCONFIG + APPSETTING_MAX_PC)
			{
				// set config here
				memcpy(cfgram->pc_config, izi_can_data->config, APPSETTING_MAX_PC);
			}
			AppConfig_Set();
			//dcbm1_settxlevel(appConfig->pc_config[APPCFG_TXLEVEL]);
			
			dcbm1_settxlevelraw(dcbm1_gettxlevel(appConfig->pl_frequency));
		
			rsp_base_setconfig.state = 0;		 // OK
		}
		else
			rsp_base_setconfig.state = -1;
		IZI_CAN_TRACE_INFO("IZI_CAN_CMD_BASE_SETCONFIG start (name: %s)\r\n", appConfig->name);
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_BASE_SETCONFIG, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_base_setconfig, sizeof(izican_rsp_base_setconfig_t), id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_BASE_GETCONFIG)
	{
		izican_rsp_base_getconfig_t rsp_base_getconfig;
		izican_req_base_getconfig_t *izi_can_data = &req->data.base_getconfig;
			
		strncpy(rsp_base_getconfig.name, appConfig->name, IZI_CAN_CMD_BASE_NAME_SIZE);
		rsp_base_getconfig.location_id = appConfig->location_id;
		memcpy(rsp_base_getconfig.config, appConfig->pc_config, APPSETTING_MAX_PC);
		
		IZI_CAN_TRACE_INFO("IZI_CAN_CMD_BASE_GETCONFIG start\r\n");
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_BASE_GETCONFIG, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_base_getconfig, IZI_CAN_CMD_BASE_MINCONFIG + APPSETTING_MAX_PC, id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_START)
	{
		int8_t res = 0;
		ticks2 = xTaskGetTickCount();
		
		if(length < sizeof(izican_req_fw_start_t))
		{
			IZI_CAN_TRACE_INFO("CMD_FW_START incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("CMD_FW_START start (%d)\r\n", length);
		}

		izican_req_fw_start_t *izi_can_data = &req->data.fw_start;
		if(res >= 0)
		{
			can_prog_offset = can_prog_address = izi_can_data->start_address;
			can_firmware_size = izi_can_data->firmware_size - can_prog_offset;
			if((izi_can_data->mem_location != fw_mem_ram) && izi_can_data->start_address < FLASH_APP_END)
			{ 
				if(can_firmware_size > FLASH_APP_MIRROR_SIZE)
					res = -4;
				/*for(uint32_t i = FLASH_APP_START; i < FLASH_APP_END; i += NVMCTRL_BLOCK_SIZE)
				{
					if(flash_block_erase(&FLASH_0, i + ((uint32_t)&mirrorflash)) != ERR_NONE)
					{
						res = -3;
						break;
					} 
					WD_Refresh();
				}*/
				can_prog_fw_mem = fw_mem_flash;
			}
			else
				res = -3;
			/*else
			{
				if(izi_can_data->start_address < RAM_APP_PROG_SIZE)
				{
					can_prog_fw_mem = fw_mem_ram;
					//memset(can_prog_ram_bfr, 0xFF, RAM_APP_PROG_SIZE);			// Clear
				}
				else
					res = -4;
			}*/
		}
		else
			res = -2;

		if(res < 0)
			can_prog_fw_mem = fw_mem_invalid;

		izican_rsp_fw_start_t rsp_fw_start;
		rsp_fw_start.result = res;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_START, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_start, sizeof(izican_rsp_fw_start_t), id.bit.seq_id);
		
		IZI_CAN_TRACE_INFO("CMD_FW_START end res: %d in %d ms (a: 0x%08x, s: 0x%08x)\r\n", res, xTaskGetTickCount() - ticks2, can_prog_address, can_firmware_size);
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_WRITE)
	{
		int8_t res = 0;
		
		if(length < sizeof(izican_req_fw_write_t))
		{
			IZI_CAN_TRACE_INFO("CMD_FW_WRITE incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("CMD_FW_WRITE start (%d)\r\n", length);
		}
		
		if(can_prog_fw_mem == fw_mem_flash)
		{
			State_SetAttentionInfo(COLOR_BLUE, COLOR_GREEN, 2, 4);
			
			if(can_prog_address < can_prog_offset)
				res = -5;
			
			if((can_prog_address + FLASH_APP_PROG_SIZE) > FLASH_APP_END)
				res = -6;
		}
		/*else if(can_prog_fw_mem == fw_mem_ram)
		{
			State_SetAttentionInfo(COLOR_MAGENTA, COLOR_BLUE, 2, 4);
			if((can_prog_address + FLASH_APP_PROG_SIZE) > RAM_APP_PROG_SIZE)
			{
				res = -7;
			}
		}*/
		else
			res = -8;

		if(res >= 0)
		{
			izican_req_fw_write_t *izi_can_data = &req->data.fw_write;
			if(can_prog_fw_mem == fw_mem_flash)
			{
				//__disable_irq();
				int32_t status;
				if((status = flash_write_page_erase(&FLASH_0, (FLASH_APP_MIRROR_BASE + can_prog_address - can_prog_offset), (uint8_t *)&izi_can_data->data[0], length)) < ERR_NONE)		// Write to flash without erase
				{
					IZI_CAN_TRACE_INFO("Page Prog failed: %d\r\n", status);
					res = -2;
				}
				else
				{
					IZI_CAN_TRACE_INFO("Page Prog OK: %d (0x%x %x %x %x %x)\r\n", status, (FLASH_APP_MIRROR_BASE + can_prog_address - can_prog_offset), izi_can_data->data[0], izi_can_data->data[1], izi_can_data->data[2], izi_can_data->data[3]);
				}
				//__enable_irq();
			}
			/*else
			{
				memcpy(&can_prog_ram_bfr[can_prog_address], &izi_can_data->data[0], length);
			}*/
		}

		IZI_CAN_TRACE_INFO("Prog page address 0x%08x @ 0x%08x res: %d (len: %d)\r\n", can_prog_address, (FLASH_APP_MIRROR_BASE + can_prog_address - can_prog_offset), res, length);

		izican_rsp_fw_write_t rsp_fw_write;
		rsp_fw_write.result = res;
		rsp_fw_write.address = can_prog_address;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_WRITE, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_write, sizeof(izican_rsp_fw_write_t), id.bit.seq_id);

		if(res >= 0)
			can_prog_address += FLASH_APP_PROG_SIZE;
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_VERIFY)
	{
		int8_t res = 0;

		if(length < sizeof(izican_req_fw_verify_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_VERIFY incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_VERIFY start (%d)\r\n", length);
		}

		//izican_req_fw_verify_t *izi_can_data = &req->data.fw_verify;
		if(can_prog_fw_mem == fw_mem_flash)
		{
			State_SetAttentionInfo(COLOR_BLUE, COLOR_GREEN, 2, 4);
			uint8_t rev = get_hw_id();
			firmware_t *fw_info = (firmware_t *)(FLASH_APP_MIRROR_BASE + can_firmware_size - (sizeof(firmare_info)));
			if(fw_info->image.start < ((uint32_t)&firmare_info) && (fw_info->image.end < (FLASH_APP_MIRROR_BASE)) && (fw_info->image.start < fw_info->image.end) &&
				((fw_info->image.device_type != SLAVE_DEVTYPE_GROUP) || (rev >= fw_info->image.hw_version_start) && (rev <= fw_info->image.hw_version_end || fw_info->image.hw_version_end == 0)))
			{
				uint16_t crc = 0x0;
				crc = Crc16Fast((uint8_t *)(fw_info->image.start + (FLASH_APP_MIRROR_BASE - can_prog_offset)), (fw_info->image.end + 1) - fw_info->image.start, crc);

				IZI_CAN_TRACE_INFO("Local CRC: 0x%04x <--> 0x%04x (start: 0x%08x, end: 0x%08x, build:%d)\r\n", crc, fw_info->image.crc, fw_info->image.start, fw_info->image.end, fw_info->version.v.build);
				if (crc != fw_info->image.crc)					// Can the app be started?
					res = -2;
			}
			else
				res = -3;
		}
		/*else if(can_prog_fw_mem == fw_mem_ram)
		{
			if(can_prog_address > FLASH_APP_FW_INFO_SIZE)
			{
				State_SetAttentionInfo(COLOR_MAGENTA, COLOR_AQUA, 2, 4);
				uint16_t crc_send = can_prog_ram_bfr[can_prog_address - 2] + (can_prog_ram_bfr[can_prog_address - 1] << 8);
				uint16_t len_send = can_prog_ram_bfr[can_prog_address - 4] + (can_prog_ram_bfr[can_prog_address - 3] << 8) + 1;

				uint16_t crc = 0x0;
				crc = Crc16Fast(can_prog_ram_bfr, len_send, crc);

				IZI_CAN_TRACE_INFO("Local CRC: 0x%04x <--> 0x%04x (start: 0x%08x, end: 0x%08x)\r\n", crc, crc_send, 0, len_send);
				if (crc != crc_send)					// Can the app be started?
					res = -2;
			}
			else
				res = -5;
		}*/
		else
			res = -6;

		izican_rsp_fw_verify_t rsp_fw_verify;
		rsp_fw_verify.result = res;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_VERIFY, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_verify, sizeof(izican_rsp_fw_verify_t), id.bit.seq_id);

		IZI_CAN_TRACE_INFO("Verify: %d\r\n", res);
		State_ClearAttentionInfo();
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_OFFSET)
	{
		int8_t res = 0;

		if(length < sizeof(izican_req_fw_offset_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_OFFSET incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_OFFSET start (%d)\r\n", length);
		}
		izican_req_fw_offset_t *izi_can_data = &req->data.fw_offset;

		if(can_prog_fw_mem == fw_mem_flash)
		{
			if(izi_can_data->address >= FLASH_APP_MIRROR_BASE)
				res = -2;
		}
		/*else if(can_prog_fw_mem == fw_mem_ram)
		{
			if(izi_can_data->address >= RAM_APP_PROG_SIZE)
				res = -3;
		}*/
		else
			res = -4;

		if(res >= 0)
			can_prog_address = izi_can_data->address;

		izican_rsp_fw_offset_t rsp_fw_offset;
		rsp_fw_offset.result = res;
		rsp_fw_offset.address = can_prog_address;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_OFFSET, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_offset, sizeof(izican_rsp_fw_offset_t), id.bit.seq_id);

		IZI_CAN_TRACE_INFO("Offset address 0x%08x\r\n", can_prog_address);
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_REBOOT)
	{
		int8_t res = 0;
		if(length < sizeof(izican_req_fw_reboot_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_REBOOT incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_REBOOT start (%d)\r\n", length);
		}
		//izican_req_fw_reboot_t *izi_can_data = &req->data.fw_reboot;

		boot_data.action = BOOT_ACTION_COPY_EXT;
		boot_data.magic = BOOT_MAGIC_VALUE;

		izican_rsp_fw_reboot_t rsp_fw_reboot;
		rsp_fw_reboot.result = res;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_REBOOT, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_reboot, sizeof(izican_rsp_fw_reboot_t), id.bit.seq_id);

		if(res >= 0)
		{
			IZI_CAN_TRACE_INFO("Reboot now\r\n");

			LOGWRITE_WARNING(ERRCODE_UPDATE_REBOOT, "Reboot for software update", log_src_task);
			IziPlus_SetMaster(0);			// Set all levels to 0
			
			vTaskDelay(250);				// Give some time to respond
			
			IziPlus_ReportShutdown();
			
			NVIC_SystemReset();
		}
	}
	else if(id.bit.command == IZI_CAN_CMD_FW_REBOOT_FAR)
	{
		int8_t res = 0;
		if(length < sizeof(izican_req_fw_reboot_far_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_REBOOT_FAR incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_FW_REBOOT_FAR start (%d)\r\n", length);
		}
		izican_req_fw_reboot_far_t *izi_can_data = &req->data.fw_reboot_far;

		if(res >= 0)
		{
			if(!IziPlus_StartUpdate(izi_can_data->device_id, can_firmware_size, izi_can_data->type))
				res = -4;
		}

		izican_rsp_fw_reboot_far_t rsp_fw_reboot_far;
		rsp_fw_reboot_far.result = res;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_FW_REBOOT_FAR, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_fw_reboot_far, sizeof(izican_rsp_fw_reboot_far_t), id.bit.seq_id);

		if(res >= 0)
		{
			IZI_CAN_TRACE_INFO("Start update module: %08x\r\n", izi_can_data->device_id);

			LOGWRITE_WARNING(ERRCODE_UPDATE_IZI, "Start izi+ software update", log_src_task);
		}
	}
	else if(id.bit.command == IZI_CAN_CMD_LINK_DISCOVER)
	{
		int8_t res = 0;
		if(length < sizeof(izican_req_link_discover_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_DISCOVER incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_DISCOVER start (%d)\r\n", length);
		}

		izican_rsp_link_discover_t rsp_link_discover;
#if IZIPLUS_DRIVER > 0
		izican_req_link_discover_t *izi_can_data = &req->data.link_discover;
		res = IziPlus_DiscoverStart(izi_can_data->options, izi_can_data->pl_frequency, izi_can_data->pl_rate, &rsp_link_discover.clear);
#elif IZI_DRIVER
		izican_req_link_discover_t *izi_can_data = &req->data.link_discover;
		res = izi_driver_discover_req(izi_can_data->options, izi_can_data->pl_frequency, izi_can_data->pl_rate) ? 0 : -1;
#else
		res = 0;
#endif
		rsp_link_discover.result = res;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_DISCOVER, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_discover, sizeof(izican_rsp_link_discover_t), id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_LINK_SETCONFIG)
	{
		int8_t res = 0;
		if(length < sizeof(izican_req_link_setconfig_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_SETCONFIG incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_SETCONFIG start (%d, %s)\r\n", length, ((izican_req_link_setconfig_t *)&req->data.link_setconfig)->txt);
		}
		izican_req_link_setconfig_t *izi_can_data = &req->data.link_setconfig;

		#if IZIPLUS_DRIVER > 0
		IziPlus_SetChannelMode(izi_can_data->serial, izi_can_data->address, izi_can_data->mode, izi_can_data->dmxfail, izi_can_data->config, 16, izi_can_data->txt) ? 0 : -1;
#elif IZI_DRIVER
		res = izi_driver_set_address(izi_can_data->module_idx, izi_can_data->address, izi_can_data->mode, izi_can_data->dmxfail, izi_can_data->config, 16, izi_can_data->txt) ? 0 : -1;
#else
		res = 0;
#endif
		izican_rsp_link_setconfig_t rsp_link_setconfig;
		rsp_link_setconfig.result = res;
		rsp_link_setconfig.serial = izi_can_data->serial;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_SETCONFIG, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_setconfig, sizeof(izican_rsp_link_setconfig_t), id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_LINK_SETIDENTIFY)
	{
		int8_t res = 0;
		if(length < sizeof(izican_req_link_setidentify_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_SETIDENTIFY incomplete data (%d)\r\n", length);
			res = -1;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_SETIDENTIFY start (%d)\r\n", length);
		}
		izican_req_link_setidentify_t *izi_can_data = &req->data.link_setidentify;
#if IZIPLUS_DRIVER > 0
		res = IziPlus_Identify(izi_can_data->serial, izi_can_data->time, izi_can_data->mode);
		if(res >= 0 && izi_can_data->time)
			can_slave_state = IZICAN_STATE_IDENTIFY;
		else if(can_slave_state == IZICAN_STATE_IDENTIFY)
			can_slave_state = IZICAN_STATE_OK;
#elif IZI_DRIVER
		res = izi_driver_set_identify(izi_can_data->module_idx, izi_can_data->time) ? 0 : -1;
#else
		res = 0;
#endif

		izican_rsp_link_setidentify_t rsp_link_setidentify;
		rsp_link_setidentify.result = res;
		rsp_link_setidentify.serial = izi_can_data->serial;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_SETIDENTIFY, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_setidentify, sizeof(izican_rsp_link_setidentify_t), id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_LINK_GETMODULE)
	{
		bool exists = false;
		if(length < sizeof(izican_req_link_getmodule_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_GETMODULE incomplete data (%d)\r\n", length);
			return;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_GETMODULE start (%d)\r\n", length);
		}
		izican_req_link_getmodule_t *izi_can_data = &req->data.link_getmodule;
		izican_rsp_link_getmodule_t rsp_link_getmodule;
#if IZIPLUS_DRIVER > 0
		uint16_t module_amount = IziPlus_ModuleAmount();
#elif IZI_DRIVER
		uint16_t module_amount = izi_get_module_amount();
#else
		uint16_t module_amount = 0;
#endif
		rsp_link_getmodule.module_idx = izi_can_data->module_idx;
		rsp_link_getmodule.module_amount = module_amount;
		if(izi_can_data->module_idx < module_amount)
		{
#if IZIPLUS_DRIVER > 0
			izi_module_t* izi_module = IziPlus_GetModule(izi_can_data->module_idx);
#elif IZI_DRIVER
			izi_module_t* izi_module = izi_get_module(izi_can_data->module_idx);
#endif
#if IZIPLUS_DRIVER > 0 || IZI_DRIVER > 0
			if(izi_module != NULL)
			{
				exists = true;
				memcpy(rsp_link_getmodule.data, (uint8_t *)&izi_module->base, sizeof(rsp_link_getmodule.data));
			}
#endif
		}
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_GETMODULE, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_getmodule, exists ? sizeof(izican_rsp_link_getmodule_t) : 4, id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_LINK_GETMONITOR)
	{
		bool exists = false;
		if(length < sizeof(izican_req_link_getmonitor_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_GETMONITOR incomplete data (%d)\r\n", length);
			return;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_LINK_GETMONITOR start (%d)\r\n", length);
		}
		izican_req_link_getmonitor_t *izi_can_data = &req->data.link_getmonitor;
		izican_rsp_link_getmonitor_t rsp_link_getmonitor;
#if IZIPLUS_DRIVER > 0
		uint16_t module_amount = IziPlus_ModuleAmount();
#elif IZI_DRIVER
		uint16_t module_amount = izi_get_module_amount();
#else
		uint16_t module_amount = 0;
#endif
		rsp_link_getmonitor.module_idx = izi_can_data->module_idx;
		if(izi_can_data->module_idx < module_amount)
		{
#if IZIPLUS_DRIVER > 0
			izi_module_t* izi_module = IziPlus_GetModule(izi_can_data->module_idx);
#elif IZI_DRIVER
			izi_module_t* izi_module = izi_get_module(izi_can_data->module_idx);
#endif
#if IZIPLUS_DRIVER > 0 || IZI_DRIVER > 0
			if(izi_module != NULL)
			{
				exists = true;
				rsp_link_getmonitor.serial_short = (uint16_t)izi_module->base.device_id;
				memcpy(rsp_link_getmonitor.data, (uint8_t *)&izi_module->var, sizeof(izi_module_var_t));
			}
#endif
		}
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_GETMONITOR, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_getmonitor, exists ? sizeof(izican_rsp_link_getmodule_t) : 2, id.bit.seq_id);
	}
	else if(id.bit.command == IZI_CAN_CMD_PRODDATA_WRITE)
	{
		bool exists = false;
		if(length < PRODDATA_MIN_SIZE)//sizeof(izican_req_link_proddata_write_t))
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_PRODDATA_WRITE incomplete data (%d)\r\n", length);
			return;
		}
		else
		{
			IZI_CAN_TRACE_INFO("IZI_CAN_CMD_PRODDATA_WRITE start (%d)\r\n", length);
		}
		izican_req_link_proddata_write_t *izi_can_data = &req->data.link_prodata_write;
		izican_rsp_link_proddata_writereport_t rsp_link_proddata_write;
		
		bool extra_info = length >= sizeof(izican_req_link_proddata_write_t);
		int8_t res = (int8_t)Production_Write(izi_can_data->serial, izi_can_data->batch, izi_can_data->plant, extra_info ? izi_can_data->time : RtcTime_GetSeconds(), extra_info ? izi_can_data->info : "");
		rsp_link_proddata_write.result = res;
		rsp_link_proddata_write.serial = izi_can_data->serial;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_PRODDATA_WRITE, IZI_CAN_MSGTYPE_RSP, (uint8_t *)&rsp_link_proddata_write, sizeof(rsp_link_proddata_write), id.bit.seq_id);
	}
}

/**
 * Report info of discovered or changed module
 */
bool izican_slave_reportmodule(uint16_t index, uint16_t total, izi_module_base_t *module, uint8_t *channels, uint8_t *config_amount, uint8_t *monitor_amount, uint8_t *monitor_status_idx, uint8_t *monitor_input_idx, uint8_t *monitor_encoder_idx)
{
	izican_req_link_modulereport_t req_modulereport;
	izican_rsp_link_modulereport_t rsp_modulereport;
	req_modulereport.module_idx = index;
	req_modulereport.module_amount = total;
	memcpy(req_modulereport.data, (uint8_t *)module, sizeof(req_modulereport.data));
	izican_extended_timeout(2000);		// 2 sec timeout (for reading json file)
	if(izican_sendcommand_req(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_MODULEREPORT, (uint8_t *)&req_modulereport, sizeof(izican_req_link_modulereport_t), (izican_response_data_t *)&rsp_modulereport) >= 0)
	{
		*channels = rsp_modulereport.channels;
		*config_amount = rsp_modulereport.config_amount;
		*monitor_amount = rsp_modulereport.monitor_amount;
		*monitor_status_idx = rsp_modulereport.monitor_status_idx;
		*monitor_input_idx = rsp_modulereport.monitor_input_idx;
		memcpy(monitor_encoder_idx, rsp_modulereport.monitor_encoder_idx, IZI_MODULE_MAX_ENCODERMAP);
		IZI_CAN_TRACE_INFO("Module report ok, devtype: %d, channels: %d, cfg: %d\r\n", module->device_type, *channels, *config_amount);

		return true;
	}
	else
		IZI_CAN_TRACE_INFO("Module report fail, idx: %d\r\n", index);
	return false;
}

/**
 * Report info of discovered or changed module
 */
bool izican_slave_reportmonitor(uint16_t index, izi_module_t *module)
{
	izican_req_link_monitorreport_t req_monitorreport;
	izican_rsp_link_monitorreport_t rsp_monitorreport;
	req_monitorreport.module_idx = index;
	req_monitorreport.serial_short = (uint16_t)module->base.device_id;
	memcpy(req_monitorreport.data, (uint8_t *)&module->var, sizeof(izi_module_var_t));
	if(izican_sendcommand_req(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_MONITORREPORT, (uint8_t *)&req_monitorreport, sizeof(izican_req_link_monitorreport_t), (izican_response_data_t *)&rsp_monitorreport) >= 0)
	{
		IZI_CAN_TRACE_DEBUG("Monitor report ok, idx: %d\r\n", index);

		return true;
	}
	else
		IZI_CAN_TRACE_INFO("Monitor report fail, idx: %d\r\n", index);
	return false;
}

bool izican_slave_reportlog(logitem_t *logitem)
{
	izican_req_dbg_logreport_t req_logreport;
	izican_rsp_dbg_logreport_t rsp_logreport;
	memcpy(req_logreport.data, (uint8_t *)logitem, sizeof(logitem_t)-2);			// Exclude CRC
	if(izican_sendcommand_req(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LOG_REPORT, (uint8_t *)&req_logreport, sizeof(izican_req_dbg_logreport_t), (izican_response_data_t *)&rsp_logreport) >= 0)
	{
		IZI_CAN_TRACE_INFO("Log report ok, serial: 0x%08x\r\n", logitem->serial);

		return true;
	}
	else
		IZI_CAN_TRACE_INFO("Log report fail, idx: 0x%08x\r\n", logitem->serial);
	return false;
}

bool izican_slave_notifylog(logitem_t *logitem)
{
	izican_req_dbg_logreport_t req_logreport;
	memcpy(req_logreport.data, (uint8_t *)logitem, sizeof(logitem_t)-2);			// Exclude CRC
	if(izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LOG_REPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&req_logreport, sizeof(izican_req_dbg_logreport_t), IZI_CAN_SEQ_AUTO) >= 0)
	{
		IZI_CAN_TRACE_INFO("Log notify ok, serial: 0x%08X\r\n", logitem->serial);

		return true;
	}
	else
		IZI_CAN_TRACE_INFO("Log notify fail, idx: %d\r\n", logitem->serial);
	return false;
}


/**
 * Report info of discovered or changed module
 */
bool izican_slave_notifymodule(uint16_t index, uint16_t total, izi_module_base_t *module)
{
	izican_req_link_modulereport_t req_modulereport;

	req_modulereport.module_idx = index;
	req_modulereport.module_amount = total;
	memcpy(req_modulereport.data, (uint8_t *)module, sizeof(req_modulereport.data));

	if(izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_MODULEREPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&req_modulereport, sizeof(izican_req_link_modulereport_t), IZI_CAN_SEQ_AUTO) >= 0)
	{
		IZI_CAN_TRACE_INFO("Module notify send: idx: %d devtype: %d\r\n", index, module->device_type);

		return true;
	}
	else
	{
		IZI_CAN_TRACE_INFO("Module notify fail: idx: %d devtype: %d\r\n", index, module->device_type);
	}
	return false;
}

/**
 * Report info of module when monitor data or online state changes
 */
bool izican_slave_notifymonitor(uint16_t index, izi_module_var_t *module)
{
	izican_req_link_monitorreport_t req_monitorreport;

	req_monitorreport.module_idx = index;
	memcpy(req_monitorreport.data, (uint8_t *)module, sizeof(izi_module_var_t));

	if(izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_MONITORREPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&req_monitorreport, (sizeof(izi_module_var_t) + 4), IZI_CAN_SEQ_AUTO) >= 0)
	{
		IZI_CAN_TRACE_INFO("Monitor notify send: idx: %d\r\n", index);

		return true;
	}
	else
	{
		IZI_CAN_TRACE_INFO("Monitor notify fail: idx: %d\r\n", index);
	}
	return false;
}

static uint16_t izican_slave_link_state;

uint16_t izican_slave_get_linkstate()
{
	return izican_slave_link_state;
}

/**
 * Report new state of link of module
 */
bool izican_slave_state(uint16_t state, uint16_t total, uint8_t pl_freq, uint8_t pl_rate, uint16_t dmx_per_sec, uint8_t izi_req_per_sec, uint8_t izi_rsp_quality, uint8_t scan_amount, uint8_t channels, uint8_t *scan_res)
{
	izican_not_link_state_t not_link_state;
	not_link_state.module_amount = total;
	not_link_state.state = state;
	not_link_state.pl_frequency = pl_freq;
	not_link_state.pl_rate = pl_rate;
	not_link_state.dmx_per_sec = dmx_per_sec;
	not_link_state.req_per_sec = izi_req_per_sec;
	not_link_state.rsp_quality = izi_rsp_quality;
	not_link_state.scan_amount = scan_amount;
	not_link_state.channels = channels;
	izican_slave_link_state = state;
	
	if(scan_amount > IZICAN_MAX_SCAN_RES)
		scan_amount = IZICAN_MAX_SCAN_RES;
		
	uint8_t length = (sizeof(izican_not_link_state_t) - IZICAN_MAX_SCAN_RES) + scan_amount;
	if(scan_amount > 0)
		memcpy(not_link_state.scan_res, scan_res, scan_amount);
	return izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_LINK_STATE, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&not_link_state, length, IZI_CAN_SEQ_AUTO) >= 0;
}

bool izican_slave_base_state(uint16_t state, uint32_t errors, uint32_t warnings, uint32_t start_info, char *txt)
{
	izican_not_base_state_t not_base_state;
	not_base_state.state = state;
	not_base_state.errors = errors;
	not_base_state.warnings = warnings;
	not_base_state.init = start_info;
	strncpy(not_base_state.txt, txt, BASE_STATE_NAME_LENGTH);
	return izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_BASE_STATE, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&not_base_state, strlen(txt) > 0 ? sizeof(izican_not_base_state_t) : BASE_STATE_MIN_LENTGH, IZI_CAN_SEQ_AUTO) >= 0;
}

bool izican_slave_monitor_report(uint8_t *values, uint8_t length)
{
	izican_not_monitor_report_t not_monitor_report;
	uint8_t len = length > MONITOR_REPORT_DATA_LENGTH ? MONITOR_REPORT_DATA_LENGTH : length;
	memcpy(not_monitor_report.values, values, len);
	not_monitor_report.length = len;
	not_monitor_report.page = 0;
	return izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_MONITOR_REPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&not_monitor_report, len + 2, IZI_CAN_SEQ_AUTO) >= 0;
}

bool izican_slave_power_report(uint8_t minute, uint8_t offset, uint8_t amount, uint16_t *data, uint32_t energy)
{
	izican_not_power_report_t not_power_report;
	not_power_report.minute = minute;
	not_power_report.amount = amount;
	not_power_report.offset = offset;
	not_power_report.total_energy = energy;
	memcpy(not_power_report.data, data, amount * 2);
	return izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_POWER_REPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&not_power_report, POWER_REPORT_MIN_LENTGH + (amount * 2), IZI_CAN_SEQ_AUTO) >= 0;
}


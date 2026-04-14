#/*
 * iziplus_driver.c
 *
 *  Created on: 22 apr. 2021
 *      Author: Milo
 */

#include "includes.h"
#include "iziplus_driver.h"
#include "dcbm1_driver.h"
#include "appconfig.h"
#include "iziplus_module_def.h"
#include "iziplus_dataframe.h"
#include "iziplus_networkframe.h"
#include "iziplus_network.h"
#include "fifo.h"
#include "rtctime.h"
#include "izi_output.h"
#include "smart_eeprom.h"
#include "version.h"
#include "state.h"
//#include "adc.h"
#include "stepdown.h"
#include "crc.h"
#include "rtctime.h"
#include "dmx_rdm_models.h"
#include "dmx_rdm_module.h"
#include "rdm.h"
#include "dmx.h"
#include "11LC160.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define TASK_PLUS_STACK_SIZE (2*1024 / sizeof(portSTACK_TYPE))
#define TASK_PLUS_TASK_PRIORITY (tskIDLE_PRIORITY + 4)

#define IDENTIFY_BUTTON_TIMEOUT		30			// Amount of seconds the button identify press will be reported
#define UPDATE_FAIL_TIMEOUT			30			// Amount of seconds a failed update will be shown

#define IZIPLUS_TIMEROS_TICKS		100			// Amount of ticks for 100 msecond timer

typedef enum iziplus_state_e {
	IZIPLUS_STATE_STARTUP			= 0,		// Possible Commissioning state 1 sec after power-up
	IZIPLUS_STATE_COMMISSIONING		= 1,		// Commissioning state entered
	IZIPLUS_STATE_COMMISSION_EXIT	= 2,		// Commissioning state ending
	IZIPLUS_STATE_NORMAL			= 3,		// Normal state
} iziplus_substate_t;

#define IZIPLUS_COMMISSIONING_TIME		2000		// Time in ms after power-up to check for discover in commissioning frequency
#define IZIPLUS_COMMISSIONING_EXTTIME	10000		// IF valid command is received in commissioning mode extend time with 10 sec
#define IZIPLUS_COM_ACTIVE_TIME			2000		// Any command in 2 sec, communication is active
#define IZIPLUS_COM_TOKEN_TIME			9000		// Every 9 seconds at least one time a token must be received that is ours

enum iziplus_cmd_delayed_state {
	DELAYED_EMPTY,
	DELAYED_ACTION,
	DELAYED_RESPONSE,
};

/*******************************************************************************
 * Globals
 ******************************************************************************/
static TaskHandle_t		xIziPlus_Task;
static TimerHandle_t  	xIziPlus_Timer = NULL;
static SemaphoreHandle_t xIziPlus_DelaySemaphore = NULL;

static uint8_t iziplus_pl_freq;
static uint8_t iziplus_pl_rate;
static uint8_t iziplus_pl_txlevel;
static uint8_t iziplus_sec_prs = 0;
static uint8_t iziplus_state;
static uint8_t iziplus_state_com_timer;
static uint8_t iziplus_session_id = 0xAA;
static int8_t iziplus_driver_ok = 0;

static bool iziplus_lightdata_ok = false;

static uint16_t iziplus_com_active_timer = IZIPLUS_COM_ACTIVE_TIME / IZIPLUS_TIMEROS_TICKS;
static uint16_t iziplus_com_tokens = IZIPLUS_COM_TOKEN_TIME / IZIPLUS_TIMEROS_TICKS;
static uint32_t iziplus_active_time = 0;
static uint16_t  iziplus_com_reinit = 0, iziplus_com_reinit_cnt = 0;

#if USE_CONTACT_TRIGGER	
static uint8_t iziplus_contact_event = 0, iziplus_contact_repeat = 0, iziplus_contact_flags, iziplus_contact_seqid, iziplus_contact_dly;
static bool iziplus_identify_local = false, iziplus_contact_req = false;
static uint32_t iziplus_contact_pan_id;
static int16_t iziplus_contact_postponed;
#else
static bool iziplus_identify_local = false;
#endif

static uint16_t iziplus_assign_network_allow = 0;

trace_level_t izi_trace_lvl = IZI_DFLT_TRACE_LVL;

char text[LOG_TEXT_SIZE + 8];		// 8 bonus...
uint8_t iziplus_nwdata[IZIPLUS_MAX_DATA];
iziplus_cmd_delayed_req_t iziplus_cmd_delayed_req = { .cmd = 0, .state = DELAYED_EMPTY };

static uint16_t iziplus_firmware_allow_time = 0;
	
static uint16_t iziplus_sleep_time = 0;	

static uint16_t iziplus_state_version_report = 4;
#ifdef EMITTER_SUPPORT
static emitter_fixture_def emitter_fixdef_copy;
#endif
extern boot_t boot_data;

static uint16_t iziplus_state_checkhw_prs = 0, iziplus_state_error_time = 0;
static bool iziplus_state_checkhw_first = false;

static iziplus_nwtemp_t iziplus_nwtemp;

/*******************************************************************************
 * Proto
 ******************************************************************************/
static void IziPlus_Task(void *p);
void IziPlus_Powerup(uint8_t commissioning);
void IziPlus_Timer100ms(TimerHandle_t xTimer);
void IziPlus_HandleRx(iziplus_data_frame_t *frame, bool handled);
bool IziPlus_ComActive();
void IziPlus_HandleRxIdle();

/*******************************************************************************
 * Code
 ******************************************************************************/
void IziPlus_Init()
{
	iziplus_nwtemp.pan_id = 0;
	
	xIziPlus_DelaySemaphore = xSemaphoreCreateBinary();
	if(xIziPlus_DelaySemaphore == NULL){
		while (1) {
			;
		}
	}
	if(xIziPlus_Task == NULL)
	{
		if (xTaskCreate(IziPlus_Task, "IziPlus", TASK_PLUS_STACK_SIZE, NULL, TASK_PLUS_TASK_PRIORITY, &xIziPlus_Task) != pdPASS) {
			while (1) {
				;
			}
		}
		if(xIziPlus_Timer == NULL)			// Init can be called multiple times
		{
			xIziPlus_Timer = xTimerCreate("IziPlusTimer", IZIPLUS_TIMEROS_TICKS, pdTRUE, ( void * ) 0, IziPlus_Timer100ms);
			if(xIziPlus_Timer == NULL)
			{
				while (1);				// Todo: what to do
			}
			else if( xTimerStart(xIziPlus_Timer, 0) != pdPASS )
			{
				while (1);				// Todo: what to do
			}
		}
		else if( xTimerStart(xIziPlus_Timer, 0) != pdPASS )
		{
			while (1);				// Todo: what to do
		}
	}
}

/*
 * 100 ms timer timed by OS. Do not call OS routines from here
 */
void IziPlus_Timer100ms(TimerHandle_t xTimer)
{
	if(++iziplus_sec_prs >= 10)
	{
		RtcTime_Tick();	
		iziplus_sec_prs = 0;
	}
	if(iziplus_state_com_timer > 0)
		iziplus_state_com_timer--;
	if(iziplus_com_active_timer > 0)			// Todo: report warning/error when no com
		iziplus_com_active_timer--;
	if(iziplus_com_tokens > 0)
		iziplus_com_tokens--;
	if(iziplus_assign_network_allow > 0)
		iziplus_assign_network_allow--;
	if(iziplus_firmware_allow_time)
		iziplus_firmware_allow_time--;
		
	IziPlus_Module_Timer100ms();
	iziplus_dataframe_timer100ms();
	dcbm1_checkerror();
	
	iziplus_active_time++;
}

extern void print_resetcause();
static volatile uint8_t iziplus_freqmode = 0;

/************************************************************************/
/* Rx handler task														        */
/************************************************************************/
static void IziPlus_Task(void *p)
{
	State_SetStartInfo(STATE_START_COMMISSIONING);
	for(int i = 0; i < 200; i++)
	{
		if(!IziPlus_Module_InitReady())			// Wait until init is ready (max 1 second)
			vTaskDelay(5);
		else
			break;
	}
	dcbm1_driver_init();
	iziplus_state = IZIPLUS_STATE_STARTUP;
	iziplus_state_com_timer = IZIPLUS_COMMISSIONING_TIME / IZIPLUS_TIMEROS_TICKS;
	IziPlus_Powerup(1);
	iziplus_freqmode = 1;
	iziplus_dataframe_init(IZIPLUS_DEVTYPE);
	iziplus_dataframe_set_rx_callback(IziPlus_HandleRx);
	iziplus_dataframe_set_rxidle_callback(IziPlus_HandleRxIdle);
	
	OS_TRACE_INFO("\r\n***** FF opstarten ******\r\nCode: %c - %d - %d (%d, %d)\r\n", boot_data.last_code_copy, boot_data.last_data_copy, boot_data.last_data2_copy, boot_data.last_code, boot_data.reset_cause);
	print_resetcause();
	boot_data.last_code = 'A';
	while(true)
	{
		if(iziplus_state != IZIPLUS_STATE_NORMAL)
		{
			if(iziplus_state_com_timer == 0)
			{
				if(iziplus_state == IZIPLUS_STATE_COMMISSION_EXIT)
					vTaskDelay(20);										// Make sure the last is send with current freq and rate
				iziplus_state = IZIPLUS_STATE_NORMAL;
				State_ClearStartInfo(STATE_START_COMMISSIONING);
				State_ClearAttentionInfo();
				IziPlus_Powerup(0);
				iziplus_freqmode = 0;
			}
			else if(iziplus_state_com_timer <= ((IZIPLUS_COMMISSIONING_TIME / IZIPLUS_TIMEROS_TICKS)/2))
			{
				if(iziplus_state != IZIPLUS_STATE_COMMISSIONING && iziplus_freqmode != 2)		// Not entered commissioning state already?
				{
					IziPlus_Powerup(2);				// Goto backup frequency
					iziplus_freqmode = 1;
				}
			}
		}
		else if(iziplus_com_reinit == 1)
		{
			dcbm1_nap(5);
			State_SetAttentionInfo(COLOR_YELLOW, COLOR_PINK, 2, 1);
			iziplus_com_reinit = 0;
		}
		
		/*else if((iziplus_com_reinit & 0x01) && iziplus_com_reinit < 8)
		{
			iziplus_driver_ok = dcbm1_init(iziplus_pl_freq, iziplus_pl_rate, iziplus_pl_txlevel);
			State_SetAttentionInfo(COLOR_YELLOW, COLOR_PINK, 3, 1);
			iziplus_com_reinit++;
		}*/
		else if(iziplus_driver_ok < 0)
		{
			IziPlus_Powerup(0);
			iziplus_freqmode = 0;
		}
		else if(!IziPlus_LightDataOk())
		{
			if(++iziplus_state_checkhw_prs > (50 * 5) || iziplus_state_checkhw_first)			// More than +/-5 sec no com? Check if chip must be reset
			{
				bool wrong_content = !dcbm1_checkrate(iziplus_pl_rate) || !dcbm1_checkfrequency(iziplus_pl_freq);
				if(wrong_content || iziplus_state_checkhw_first)		// Check if chip is at the expected frequency and rate
				{
					IziPlus_Powerup(0);					// Reset Yamar
					if(wrong_content)					
					{
						State_SetError(STATE_ERROR_HW_ERROR);	// Show error for at least 4 sec if content in Yamar was not ok (reset?)
						iziplus_state_error_time = 4 * 50;
					}
				}
				iziplus_state_checkhw_prs = 0;
				iziplus_state_checkhw_first = false;		
			}
		}
		else 
		{
			iziplus_state_checkhw_prs = 0; 
			iziplus_state_checkhw_first = true;
			if(iziplus_state_error_time > 0)					// If showing hardware error, check if time expired while com is ok
			{
				if(--iziplus_state_error_time == 0)
					State_ClearError(STATE_ERROR_HW_ERROR);
			}
		}
		
		boot_data.last_code = 'B';
		xSemaphoreTake(xIziPlus_DelaySemaphore, 20);					// Wait max 20ms. It can be given when action needs to be immediately executed
		TickType_t quality_ticks_now = xTaskGetTickCount();
		
		if(iziplus_cmd_delayed_req.state == DELAYED_ACTION)
		{
			boot_data.last_code = 'H';
			if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_FW_WRITE)
			{
				int32_t status;
				if((status = flash_write_page_erase(&FLASH_0, iziplus_cmd_delayed_req.address, iziplus_cmd_delayed_req.data, iziplus_cmd_delayed_req.length)) < ERR_NONE)		// Write to flash and only erase if new sector
				{
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_FAIL;
				}
				else
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_OK;
					
				iziplus_cmd_delayed_req.state = iziplus_cmd_delayed_req.msg_type.u.frame_type == IZIPLUS_FRAMETYPE_REQ ? DELAYED_RESPONSE : DELAYED_EMPTY;
				iziplus_cmd_delayed_req.time = xTaskGetTickCount() - quality_ticks_now;
			}
			else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_FW_APPLY)
			{
				uint16_t crc = 0x0;
				crc = Crc16Fast((uint8_t *)iziplus_cmd_delayed_req.address, iziplus_cmd_delayed_req.length, crc);
				
				firmware_t *fw_info = (firmware_t *)((FLASH_APP_MIRROR_BASE - FLASH_APP_MIRROR_SHIFT) + ((uint32_t)&firmare_info));					// Read firmware info of copy
				IZI_TRACE_INFO("Local CRC: 0x%04x <--> 0x%04x (start: 0x%08x, end: 0x%08x, addr: 0x%08x, len: 0x%08x)\r\n", crc, fw_info->image.crc, fw_info->image.start, fw_info->image.end, iziplus_cmd_delayed_req.address, iziplus_cmd_delayed_req.length);
				if (crc != fw_info->image.crc)					// Can the app be started?
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_CORRUPT_MEMORY;
				else
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_OK;
				iziplus_cmd_delayed_req.state = DELAYED_RESPONSE;
				iziplus_cmd_delayed_req.time = xTaskGetTickCount() - quality_ticks_now;
			}
			else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_TEXT || iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_CONFIG || iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_ASSIGN_CHANMODE)
			{
				appconfig_t *appconfig = AppConfig_GetPtr();				// Get current app config in RAM
				bool res = AppConfig_Set(appconfig);						// Write back
				if(res)
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_OK;
				else
					iziplus_cmd_delayed_req.tokenresult = IZIPLUS_TOKERES_CORRUPT_MEMORY;			// Write failed
					
				iziplus_cmd_delayed_req.state = DELAYED_RESPONSE;
				iziplus_cmd_delayed_req.time = xTaskGetTickCount() - quality_ticks_now;
			}
		}		
		/*else if(iziplus_sleep_time > 0)
		{
			dcbm1_sleep();
			vTaskDelay(iziplus_sleep_time);
			iziplus_sleep_time = 0;
			dcbm1_wake();
			//gpio_set_pin_level(LED_R, true);
		}*/
		
		boot_data.last_code = 'C';
#ifdef DMX_RDM		
		if(iziplus_com_active_timer == 0 && xTaskGetTickCount() > 10000 && !Dmx_Valid())		// Wait at least 10 sec after power-up
#else
		if(iziplus_com_active_timer == 0 && xTaskGetTickCount() > 10000)		// Wait at least 10 sec after power-up
#endif				
			State_SetError(STATE_ERROR_NO_COM);
		else
			State_ClearError(STATE_ERROR_NO_COM);

#ifdef DMX_RDM			
		if(iziplus_com_tokens == 0 && xTaskGetTickCount() > 30000 && !Dmx_Valid())				// Wait at least 30 sec after power-up (else error will appear after long scan)
#else
		if(iziplus_com_tokens == 0 && xTaskGetTickCount() > 30000)				// Wait at least 30 sec after power-up (else error will appear after long scan)
#endif		
			State_SetError(STATE_ERROR_INCOMPLETE_COM);
		else
			State_ClearError(STATE_ERROR_INCOMPLETE_COM);
			
		REPORT_STACK(TASK_PLUS_STACK_SIZE, TASK_IZIPLUS);
	}
}

bool IziPlus_LightDataOk()
{
#ifdef DMX_RDM	
	return (iziplus_lightdata_ok && iziplus_com_active_timer > 0) || Dmx_Valid();
#else
	return (iziplus_lightdata_ok && iziplus_com_active_timer > 0);
#endif		
}

uint32_t iziplus_data_get_panid()
{
	if(iziplus_nwtemp.pan_id > 0)
		return iziplus_nwtemp.pan_id;
	return appConfig->pan_id;
}

uint8_t iziplus_data_get_frequency()
{
	if(iziplus_nwtemp.pan_id > 0)
		return iziplus_nwtemp.pl_frequency;
	return appConfig->pl_frequency;
}

uint8_t iziplus_data_get_rate()
{
	if(iziplus_nwtemp.pan_id > 0)
		return iziplus_nwtemp.pl_rate;
	return appConfig->pl_rate;
}

uint8_t iziplus_data_get_short_id()
{
	if(iziplus_nwtemp.pan_id > 0)
		return iziplus_nwtemp.short_id;
	return appConfig->short_id;
}

uint8_t iziplus_data_get_tx_level()
{
	if(iziplus_nwtemp.pan_id > 0)
		return iziplus_nwtemp.pl_txlevel;
	return appConfig->pl_txlevel;
}

void IziPlus_HandleRx(iziplus_data_frame_t *frame, bool handled)
{
	boot_data.last_code = 'D';
	if(handled)
	{
		bool cmd_ok = false;
		if((frame->pan_id & IZIPLUS_PANID_BASE_SERIAL) == (iziplus_data_get_panid() & IZIPLUS_PANID_BASE_SERIAL))			// Recognize lower part of pan id? Missed a discover?
		{
			iziplus_network_data_t *nw_data = ((iziplus_network_data_t *)frame->data);
			if(frame->destination == 0)
			{
				if(((nw_data->seqid == (PRODUCTION_SERIAL & 0xFF)) || (nw_data->seqid == ((PRODUCTION_SERIAL + 0x79) & 0xFF))) && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT) && (nw_data->tokenres == IZIPLUS_TOKEN_FREE))								// Report module missed a discover and wants to join (limit it by searching for special seqid (odd and even possibility))
				{
					//State_SetAttentionInfo(COLOR_GREEN, COLOR_AQUA, 10, 1);
					GEN_TRACE_INFO("Join Pan: %08X, our PAN: %08X\r\n", frame->pan_id, appConfig->pan_id);
					
					izi_version_u version = { .v.major = IZIPLUS_PROTOCOL_VERSION_MAJOR, .v.minor = IZIPLUS_PROTOCOL_VERSION_MINOR };
					uint16_t nw_length = iziplus_networkframe_joinnetworkaction(iziplus_nwdata, nw_data->seqid, iziplus_data_get_panid(), version);
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
				}
			}
			else if(frame->destination == PRODUCTION_SERIAL)		// Is it our serial? Check if should respond if 24-bit of Pan id is OK
			{
				if(nw_data->cmd == IZIPLUS_CMD_ASSIGN_NETWORK)		// Network correction is allowed
				{
					cmd_ok = true;				
					iziplus_assign_network_allow = 5;				// Allow max 500ms (so also retries are accepted when response is not received)
				}
			}
		}
		
		if(!cmd_ok)
			return;
	}
	
	int8_t tokenresult = IZIPLUS_TOKERES_OK;
	iziplus_network_data_t *nw_data = ((iziplus_network_data_t *)frame->data);
	if(nw_data->msgtype.u.sync)
	{
		State_TimerSync();
		IziPlus_Module_IdentifySync();
	}
	
	boot_data.last_code = 'E';
	//OS_TRACE_INFO("IZI command: 0x%02x, ftype: 0x%02x, ctype: 0x%02x\r\n", nw_data->cmd, nw_data->msgtype.u.frame_type, nw_data->msgtype.u.cmd_type);
	
	if(nw_data->cmd == IZIPLUS_CMD_DISCOVER && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_ACTION && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_discover_req_t)))
		{
#ifndef DEBUG			
			if(iziplus_state == IZIPLUS_STATE_COMMISSIONING)
#endif			
			{
				iziplus_cmd_discover_req_t *discover_req = (iziplus_cmd_discover_req_t *)&frame->data[sizeof(iziplus_network_data_t)];
				if(iziplus_session_id != discover_req->session_id)					// If same session as before, do not reply
				{
					uint16_t random_wait = (rand_sync_read32(&RAND_0) & 0xFF) + (xTaskGetTickCount() & 0x7F);			// Random number between 0 and 384 (ms)
					vTaskDelay(random_wait);
					izi_version_u version = { .v.major = IZIPLUS_PROTOCOL_VERSION_MAJOR, .v.minor = IZIPLUS_PROTOCOL_VERSION_MINOR };
					uint16_t nw_length = iziplus_networkframe_discover(iziplus_nwdata, nw_data->seqid, version);		// Respond with current protocol version
					/*uint16_t x =*/ iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
					//OS_TRACE_INFO("Discover: %d [%d - %d] (%d %d) dt: 0x%02X\r\n", iziplus_state, iziplus_session_id, ((iziplus_cmd_discover_req_t *)&frame->data[sizeof(iziplus_network_data_t)])->session_id, random_wait, x, IZIPLUS_DEVTYPE);
				}
			}
		}
	}
	else if(nw_data->cmd == IZIPLUS_CMD_ASSIGN_NETWORK && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ || nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT))
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			//State_SetAttentionInfo(COLOR_MAGENTA, COLOR_AQUA, 10, 4);
#ifndef DEBUG			
			if((iziplus_state == IZIPLUS_STATE_COMMISSIONING) || (iziplus_assign_network_allow))
			{
#endif
				if(frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_assign_network_wreq_t)))
				{
					iziplus_cmd_assign_network_wreq_t *assign_network_req = (iziplus_cmd_assign_network_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
					// Only accept when a request, in case of notification, check if the panid differs (new PowerCom) or the frequency differs (new channel). Notifications only used in case the response to DISCOVER or ASSIGN_NETWORK does not reach the PowerCom
					if((nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ) || ((iziplus_data_get_panid() & IZIPLUS_PANID_BASE_SERIAL) != (assign_network_req->pan_id & IZIPLUS_PANID_BASE_SERIAL)) || (iziplus_data_get_frequency() != assign_network_req->freq))	// Also allow notification broadcast
					{
						bool res = true;
						if(nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT)
							State_SetAttentionInfo(COLOR_MAGENTA, COLOR_YELLOW, 10, 4);
							
						if(assign_network_req->options != IZIPLUS_OPTION_ASSIGN_NETWORK_TEMP)		// Not temporary?
						{
							appconfig_t *appconfig = AppConfig_Get();					// Get current app config in RAM
							appconfig->pan_id = assign_network_req->pan_id;				// Set all received network config
							appconfig->pl_frequency = assign_network_req->freq;
							appconfig->pl_rate = assign_network_req->rate;
							appconfig->short_id = assign_network_req->shortid;
							appconfig->pl_txlevel = assign_network_req->txlevel;
							res = AppConfig_Set(appconfig);						// Write back
						}
						else
						{
							iziplus_nwtemp.pan_id = assign_network_req->pan_id;
							iziplus_nwtemp.pl_frequency = assign_network_req->freq;
							iziplus_nwtemp.pl_rate = assign_network_req->rate;
							iziplus_nwtemp.pl_txlevel = assign_network_req->txlevel;
							iziplus_nwtemp.short_id = assign_network_req->shortid;
						}
						if(res)
						{
							iziplus_session_id = assign_network_req->session_id;	
							if((nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT))		
							{
								uint16_t nw_length = iziplus_networkframe_assign_network(iziplus_nwdata, nw_data->seqid);			// Send response OK
								iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
							}
						}
						else if((nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT))
							tokenresult = IZIPLUS_TOKERES_CORRUPT_MEMORY;			// Write failed
					}
				}
				else if((nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT))
					tokenresult = IZIPLUS_TOKERES_INVALID_DATA;					// Incorrect content (not enough parameters)
#ifndef DEBUG	
			}
			else if((nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT))
				tokenresult = IZIPLUS_TOKERES_FAIL;					// Not in commissioning mode
#endif
		}
		else if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_assign_network_rreq_t)))
		{
			// Todo: Implement
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_ASSIGN_CHANMODE && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			if(frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_assign_chanmode_wreq_t)))
			{
				iziplus_cmd_assign_chanmode_wreq_t *assign_channelmode_req = (iziplus_cmd_assign_chanmode_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
				if(assign_channelmode_req->mode < MODE_AMOUNT)
				{
					appconfig_t *appconfig = AppConfig_Get();						// Get current app config in ram
					if(appconfig->channel != assign_channelmode_req->channel_id)
					{
						appconfig->channel = assign_channelmode_req->channel_id;		// Change all channel mode parameters
#ifdef DMX_RDM
						DmxRdm_Module_AddQueuedMessage(0, 0, E120_DMX_START_ADDRESS, NULL, 0);
#endif						
					}
					if(appconfig->mode != assign_channelmode_req->mode)
					{
						appconfig->mode = assign_channelmode_req->mode;
#ifdef DMX_RDM
						DmxRdm_Module_AddQueuedMessage(0, 0, E120_DMX_PERSONALITY, NULL, 0);
#endif
					}
					appconfig->offset = assign_channelmode_req->offset;
					appconfig->dmxfail = assign_channelmode_req->dmxfail;
					
					uint16_t nw_length = iziplus_networkframe_assign_channelmodewritedelay(iziplus_nwdata, nw_data->seqid, IziPlus_Module_GetChannelAmountTemp());			// Send response delayed, write smart eeprom can take too long (most of the time it is quick)
					iziplus_cmd_delayed_req.cmd = IZIPLUS_CMD_ASSIGN_CHANMODE;
					iziplus_cmd_delayed_req.state = DELAYED_ACTION;
					iziplus_cmd_delayed_req.length = 1;
					iziplus_cmd_delayed_req.msg_type.b = nw_data->msgtype.b;
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
						
					if(iziplus_cmd_delayed_req.state == DELAYED_ACTION)
						xSemaphoreGive(xIziPlus_DelaySemaphore);
				}
				else
					tokenresult = IZIPLUS_TOKERES_INVALID_DATA;						// Write failed
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;							// Incorrect content (not enough parameters)
		}
		else if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_assign_chanmode_rreq_t)))
		{
			//iziplus_cmd_assign_chanmode_wreq_t *assign_channelmode_req = (iziplus_cmd_assign_chanmode_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
			uint16_t nw_length = iziplus_networkframe_assign_channelmoderead(iziplus_nwdata, nw_data->seqid, appConfig->channel, appConfig->mode, appConfig->dmxfail, appConfig->offset, IziPlus_Module_GetChannelAmount());
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_CONFIG && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			if(frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + 1))//sizeof(iziplus_cmd_config_wreq_t)))
			{
				uint8_t config_amount = frame->lengthdir.u.length - sizeof(iziplus_network_data_t);
				if(config_amount > 0 && config_amount <= CONFIG_SIZE)
				{
					iziplus_cmd_config_wreq_t *config_write_req = (iziplus_cmd_config_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
					uint8_t offset = 0;
							
					for(int i = 0; i < config_amount; i++)				// Check if all config parameters are within permitted limits
					{
						if(i + offset >= CONFIG_SIZE)
							break;
								
						uint16_t data = iziplus_fixdef.configs[i].size == 2 ? (config_write_req->config[i + offset] + (config_write_req->config[i+1+offset] << 8)) : config_write_req->config[i+offset];
						if(data < iziplus_fixdef.configs[i].min || data > iziplus_fixdef.configs[i].max)
						{
							tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
							break;
						}
						offset += (iziplus_fixdef.configs[i].size - 1);			// Add xtra when 16-bit
					}
					if(tokenresult == IZIPLUS_TOKERES_OK)
					{
						//appconfig_t *appconfig = AppConfig_Get();		// Get current settings in ram
						//memcpy(appconfig->config, config_write_req->config, config_amount);		// Change config
						IziPlus_Module_SetConfigByIndex(0, config_write_req->config, config_amount);
						
						uint16_t nw_length = iziplus_networkframe_assign_configwritedly(iziplus_nwdata, nw_data->seqid);			// Send response delayed, write smart eeprom can take too long (most of the time it is quick)
						iziplus_cmd_delayed_req.cmd = IZIPLUS_CMD_CONFIG;
						iziplus_cmd_delayed_req.state = DELAYED_ACTION;
						iziplus_cmd_delayed_req.length = config_amount;
						iziplus_cmd_delayed_req.msg_type.b = nw_data->msgtype.b;
						
						iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
						if(iziplus_cmd_delayed_req.state == DELAYED_ACTION)
							xSemaphoreGive(xIziPlus_DelaySemaphore);
					}
				}
				else
					tokenresult = IZIPLUS_TOKERES_INVALID_DATA;				// Incorrect content (not enough parameters)
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;					// Incorrect content (not enough parameters)
		}
		else if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_config_rreq_t)))
		{
			uint16_t nw_length = iziplus_networkframe_assign_configread(iziplus_nwdata, nw_data->seqid, IziPlus_Module_GetConfig(), CONFIG_SIZE);
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_TEXT && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			if(frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + 1))//sizeof(iziplus_cmd_text_wreq_t)))
			{
				uint8_t text_amount = frame->lengthdir.u.length - sizeof(iziplus_network_data_t);
				if(text_amount <= NAME_SIZE)
				{
					iziplus_cmd_text_wreq_t *text_write_req = (iziplus_cmd_text_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
						
					if(text_amount < (NAME_SIZE - 1))
					{
						text_write_req->text[text_amount] = 0;							// Make sure it is null terminated
						text_amount++;
					}
					
					appconfig_t *appconfig = AppConfig_Get();							// Get current settings in ram
					if(memcmp(appconfig->name, text_write_req->text, text_amount) != 0)		// Check if not the same
					{
						memcpy(appconfig->name, text_write_req->text, text_amount);			// Change text
#ifdef DMX_RDM
						DmxRdm_Module_AddQueuedMessage(0, 0, E120_DEVICE_LABEL, NULL, 0);
#endif						
					}
					
					uint16_t nw_length = iziplus_networkframe_textwritedly(iziplus_nwdata, nw_data->seqid);			// Send response delayed, write smart eeprom can take too long (most of the time it is quick)
					iziplus_cmd_delayed_req.cmd = IZIPLUS_CMD_TEXT;
					iziplus_cmd_delayed_req.state = DELAYED_ACTION;
					iziplus_cmd_delayed_req.length = text_amount;
					iziplus_cmd_delayed_req.msg_type.b = nw_data->msgtype.b;
					
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
					if(iziplus_cmd_delayed_req.state == DELAYED_ACTION)
						xSemaphoreGive(xIziPlus_DelaySemaphore);					
				}
				else
					tokenresult = IZIPLUS_TOKERES_INVALID_DATA;					// Incorrect content (not enough parameters)
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
		}
		else if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_text_rreq_t)))
		{
			uint8_t txt_len = strlen(appConfig->name) + 1;
			if(txt_len > NAME_SIZE)
				txt_len = NAME_SIZE;
				
			uint16_t nw_length = iziplus_networkframe_textread(iziplus_nwdata, nw_data->seqid, (uint8_t *)appConfig->name, txt_len);
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_VERSIONS && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_versions_rreq_t)))
		{
			izi_version_u protversion = { .v.major = IZIPLUS_PROTOCOL_VERSION_MAJOR, .v.minor = IZIPLUS_PROTOCOL_VERSION_MINOR };
			// Send all version and hw parameters
			uint16_t nw_length = iziplus_networkframe_versionread(iziplus_nwdata, nw_data->seqid, IziPlus_Module_GetAppVersion(), IziPlus_Module_GetBootVersion(), protversion, IziPlus_Module_GetTypedefVersion(),
#ifdef EMITTER_SUPPORT			
				IziPlus_Module_GetImageVersion(), IziPlus_Module_GetLedtableVersion(), IziPlus_Module_GetMemoryVersion(), IziPlus_Module_GetHwRevision(), IziPlus_Module_GetVariant(), false, emitter_fixdef.hwref, emitter_proddata.serial);
#else
				IziPlus_Module_GetImageVersion(), IziPlus_Module_GetLedtableVersion(), IziPlus_Module_GetMemoryVersion(), IziPlus_Module_GetHwRevision(), IziPlus_Module_GetVariant(), false, 0, 0);
#endif
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_IDENTIFY && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ || nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT))
	{
		if((nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_ACTION) && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_identify_req_t)))
		{
			iziplus_cmd_identify_req_t *identify_req = (iziplus_cmd_identify_req_t *)&frame->data[sizeof(iziplus_network_data_t)];
			IziPlus_Module_SetIdentify(identify_req->mode, identify_req->time);
			if(nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT)
			{
				uint16_t nw_length = iziplus_networkframe_identifyaction(iziplus_nwdata, nw_data->seqid);
				iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
			}
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_OPERATION_MODE && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ || nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT))
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{			
			iziplus_cmd_operation_mode_req_t *operationmode_write_req = (iziplus_cmd_operation_mode_req_t *)&frame->data[sizeof(iziplus_network_data_t)];
			OS_TRACE_INFO("Operation mode: %d (%d)\r\n", operationmode_write_req->mode, iziplus_state); 
			if(operationmode_write_req->mode == IZIPLUS_OPERATIONMODE_NORMAL)
			{
				iziplus_state = IZIPLUS_STATE_COMMISSION_EXIT;
				iziplus_state_com_timer = 0;					// Stop pending commissioning
			}
#if DEBUG			
			else 
#else
			else if(iziplus_state == IZIPLUS_STATE_STARTUP)		// Still 1 sec after power-up?
#endif
			{
				iziplus_state = IZIPLUS_STATE_COMMISSIONING;	// Enter commissioning state
				State_SetAttentionInfo(COLOR_GREEN, COLOR_AQUA, 10, 2);
			}
			
			if(nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)		// Only respond on request
			{
				uint16_t nw_length = iziplus_networkframe_operationmodewrite(iziplus_nwdata, nw_data->seqid, iziplus_data_get_frequency(), iziplus_data_get_rate(), iziplus_data_get_short_id());
				iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
			}
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(((nw_data->cmd == IZIPLUS_CMD_LIGHT_DATA) || (nw_data->cmd == IZIPLUS_CMD_LIGHT_DATA_NOINPUT)) && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT))
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			uint8_t channel_amount = IziPlus_Module_GetChannelAmount();
			if(frame->lengthdir.u.length >= ((appConfig->offset + channel_amount) + 4))			// Enough bytes for out modes channel amount and offset?
			{
				Izi_OutputSetBuffer(IZIOUTPUT_SRC_IZI, 0, &frame->data[sizeof(iziplus_network_data_t) + appConfig->offset], channel_amount);		// Set new levels
				if(nw_data->cmd == IZIPLUS_CMD_LIGHT_DATA_NOINPUT)
				{
#ifdef DMX_RDM					
					if(!Dmx_Valid())
#endif					
						State_ComToggleNoData();
					iziplus_lightdata_ok = false;
				}
				else
				{
#ifdef DMX_RDM					
					if(!Dmx_Valid())
#endif
					{
						if(iziplus_nwtemp.pan_id > 0)
							State_ComToggleTempNetwork();
						else
							State_ComToggle();
					}
					iziplus_lightdata_ok = true;
				}
				State_ClearWarning(STATE_WARNING_DMX_WARN);
			}
			else
				State_SetWarning(STATE_WARNING_DMX_WARN);
		}
	}
	else if(nw_data->cmd == IZIPLUS_CMD_FW_WRITE && (nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ || nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT))
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE)
		{
			if(nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT || iziplus_firmware_allow_time > 0)
			{
				iziplus_cmd_fw_write_wreq_t *firmware_write_req = (iziplus_cmd_fw_write_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
				if((firmware_write_req->address + (frame->lengthdir.u.length - 8)) <= (FLASH_APP_END - FLASH_APP_START))		// Check if in valid area (-8 is networkdata + 4 bytes address)
				{
					int32_t status;
					uint16_t nw_length;
					//StepDown_Close(1000);
					State_SetAttentionInfo(COLOR_GREEN, COLOR_BLUE, 2, 2);
					if(!flash_check_erase(FLASH_APP_MIRROR_BASE + firmware_write_req->address))
					{
						if((status = flash_write_page_erase(&FLASH_0, (FLASH_APP_MIRROR_BASE + firmware_write_req->address), (uint8_t *)&firmware_write_req->data[0], (frame->lengthdir.u.length - 8))) < ERR_NONE)		// Write to flash and only erase if new sector
						{
							//IZI_TRACE_INFO("Page Prog failed: %d\r\n", status);
							tokenresult = IZIPLUS_TOKERES_FAIL;
						}
						else
						{
							IZI_TRACE_INFO("Page Prog OK: %d %d (0x%x %x %x %x %x)\r\n", status, frame->lengthdir.u.length - 8, (FLASH_APP_MIRROR_BASE + firmware_write_req->address), firmware_write_req->data[0], firmware_write_req->data[1], firmware_write_req->data[2], firmware_write_req->data[3]);
						}
						nw_length = iziplus_networkframe_firmwarewrite(iziplus_nwdata, nw_data->seqid);				// Send response
					}
					else 
					{
						nw_length = iziplus_networkframe_firmwarewritedly(iziplus_nwdata, nw_data->seqid);			// Send response delayed, erase will take too long
						iziplus_cmd_delayed_req.cmd = IZIPLUS_CMD_FW_WRITE;
						iziplus_cmd_delayed_req.state = DELAYED_ACTION;
						iziplus_cmd_delayed_req.address = (FLASH_APP_MIRROR_BASE + firmware_write_req->address);
						iziplus_cmd_delayed_req.length = (frame->lengthdir.u.length - 8);
						iziplus_cmd_delayed_req.msg_type.b = nw_data->msgtype.b;
						memcpy(iziplus_cmd_delayed_req.data, (uint8_t *)&firmware_write_req->data[0], (frame->lengthdir.u.length - 8));
					}
					if(nw_data->msgtype.u.frame_type != IZIPLUS_FRAMETYPE_NOT)
						iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);	
					if(iziplus_cmd_delayed_req.state == DELAYED_ACTION)
						xSemaphoreGive(xIziPlus_DelaySemaphore);
				}
				else
					tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
			}
			else if(nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
				tokenresult = IZIPLUS_TOKERES_NOT_ALLOWED;
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_FW_APPLY && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_ACTION)
		{
			//iziplus_cmd_fw_apply_wreq_t *firmware_write_req = (iziplus_cmd_fw_apply_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
			firmware_t *fw_info = (firmware_t *)((FLASH_APP_MIRROR_BASE - FLASH_APP_MIRROR_SHIFT) + ((uint32_t)&firmare_info));					// Read firmware info of copy
			uint8_t rev = IziPlus_Module_GetHwRevision();
			if ((fw_info->image.marker == IMAGE_MARKER) &&
				(fw_info->image.device_type <= IZIPLUS_DEVTYPE) &&
				((fw_info->image.device_type + fw_info->image.device_type_range) > IZIPLUS_DEVTYPE) &&
				(fw_info->image.start < ((uint32_t)&firmare_info)) &&
				(fw_info->image.end < (FLASH_APP_MIRROR_BASE)) &&
				(fw_info->image.start < fw_info->image.end) &&
				((rev >= (uint8_t)fw_info->image.hw_version_start) &&
				(((uint8_t)fw_info->image.hw_version_end == 0) ||
				(rev <= (uint8_t)fw_info->image.hw_version_end))))											// Check start and end if it makes sense and if suited for this hw revision
			{
				iziplus_cmd_delayed_req.cmd = IZIPLUS_CMD_FW_APPLY;
				iziplus_cmd_delayed_req.state = DELAYED_ACTION;
				iziplus_cmd_delayed_req.address = (fw_info->image.start + (FLASH_APP_MIRROR_BASE - FLASH_APP_MIRROR_SHIFT));
				iziplus_cmd_delayed_req.length = (fw_info->image.end + 1) - fw_info->image.start;
				iziplus_cmd_delayed_req.msg_type.b = nw_data->msgtype.b;
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
				
			OS_TRACE_INFO("Fw Apply: %d type: %0x04X, itype: %04X %d, rev: %d\r\n", tokenresult, IZIPLUS_DEVTYPE, fw_info->image.device_type, fw_info->image.device_type_range, rev);
				
			if(tokenresult == IZIPLUS_TOKERES_OK)
			{
				uint16_t nw_length = iziplus_networkframe_firmwareapplydly(iziplus_nwdata, nw_data->seqid);
				iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
				
				xSemaphoreGive(xIziPlus_DelaySemaphore);
			}
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_FW_ALLOW && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_ACTION && frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + sizeof(iziplus_cmd_fw_allow_wreq_t)))
		{
			iziplus_cmd_fw_allow_wreq_t *firmware_allow_req = (iziplus_cmd_fw_allow_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
			izi_versionplus_u appversion = IziPlus_Module_GetAppVersion();
			if(firmware_allow_req->app_version.d != appversion.d)			// Versions not equal, report OK
			{
				iziplus_firmware_allow_time = firmware_allow_req->time * (configTICK_RATE_HZ / IZIPLUS_TIMEROS_TICKS);
				
				int16_t nw_length = iziplus_networkframe_firmwareallow(iziplus_nwdata, nw_data->seqid, appversion);
				iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
			}
			else
				tokenresult = IZIPLUS_TOKERES_VERSION_EQUAL;				// Report updating does not make a difference
		}
		else
			tokenresult = IZIPLUS_TOKERES_INVALID_CMDTYPE;
	}
	else if(nw_data->cmd == IZIPLUS_CMD_PRODUCTION_DATA && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE && frame->lengthdir.u.length >= (16 + 4))
		{
			iziplus_cmd_production_data_wreq_t *production_data_req = (iziplus_cmd_production_data_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
			
			proddata_t proddata;
			uint16_t length = frame->lengthdir.u.length - 4;
			if(length <= sizeof(proddata_t))
			{
				memcpy(&proddata, production_data_req->data, length);
				if(length <= 16)
				{
					proddata.time = RtcTime_GetSeconds();
					strcpy(proddata.info, "-");
				}
				if(Production_WriteStruct(&proddata) >= 0)
				{
					uint16_t nw_length = iziplus_networkframe_production_data_write(iziplus_nwdata, nw_data->seqid);
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
					State_SetWarning(STATE_WARNING_REBOOT_NEEDED);
				}
				else
					tokenresult = IZIPLUS_TOKERES_FAIL;
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
		}
		else if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ && frame->lengthdir.u.length >= (16 + 4))
		{
			uint16_t nw_length = iziplus_networkframe_production_data_read(iziplus_nwdata, nw_data->seqid, (uint8_t *)PRODUCTION_DATA, sizeof(proddata_t));
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
	}
#ifdef EMITTER_SUPPORT	
	else if(nw_data->cmd == IZIPLUS_CMD_EMITTER_DATA && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_WRITE && frame->lengthdir.u.length >= (16 + 4))
		{
			iziplus_cmd_emitter_data_wreq_t *emitter_data_req = (iziplus_cmd_emitter_data_wreq_t *)&frame->data[sizeof(iziplus_network_data_t)];
			
			uint16_t length = frame->lengthdir.u.length - 4;
			if(length >= 8 && (emitter_data_req->address + emitter_data_req->length) <= sizeof(emitter_fixture_def))		// First 256 bytes has CRC16 check
			{
				memcpy((((uint8_t *)&emitter_fixdef_copy + emitter_data_req->address)), emitter_data_req->data, emitter_data_req->length);
				if(emitter_data_req->lock & 0x80)			// Last byte for writing
				{
					uint16_t crc = Crc16((uint8_t *)&emitter_fixdef_copy, sizeof(emitter_fixdef_copy) - 2, 0);
					if(crc == emitter_fixdef_copy.crc && emitter_fixdef_copy.max_channels <= MAX_DIM_CHANNELS &&
						emitter_fixdef_copy.device_type == iziplus_devtype_base)		// Check CRC and content
					{
						M11LC160_WriteLock(0x00);
						if(M11LC160_WriteMem((uint8_t *)&emitter_fixdef_copy, EXT_EEPROM_ADRR_FIXTURE, sizeof(emitter_fixdef_copy)) >= 0)
						{
							uint16_t nw_length = iziplus_networkframe_emitter_data_write(iziplus_nwdata, nw_data->seqid);
							iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
							State_SetWarning(STATE_WARNING_REBOOT_NEEDED);
							
							if((emitter_data_req->lock & 0x3F) > 0)
								M11LC160_WriteLock(emitter_data_req->lock & 0x3F);
						}
						else
						{
							tokenresult = IZIPLUS_TOKERES_FAIL;
							State_SetError(STATE_ERROR_NO_EMITTER);
						}
					}
					else
						tokenresult = IZIPLUS_TOKERES_FAIL;
				}
				else
				{
					uint16_t nw_length = iziplus_networkframe_emitter_data_write(iziplus_nwdata, nw_data->seqid);
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
				}
			}
			else if(length >= 8 && (emitter_data_req->address + emitter_data_req->length) <= EXT_EEPROM_SIZE)
			{
				M11LC160_WriteLock(0x00);
				if(M11LC160_WriteMem(emitter_data_req->data, emitter_data_req->address, emitter_data_req->length) >= 0)
				{
					uint16_t nw_length = iziplus_networkframe_emitter_data_write(iziplus_nwdata, nw_data->seqid);
					iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
					State_SetWarning(STATE_WARNING_REBOOT_NEEDED);
					
					if((emitter_data_req->lock & 0x3F) > 0)
						M11LC160_WriteLock(emitter_data_req->lock & 0x3F);
				}
				else
				{
					tokenresult = IZIPLUS_TOKERES_FAIL;
					State_SetError(STATE_ERROR_NO_EMITTER);
				}
			}
			else
				tokenresult = IZIPLUS_TOKERES_INVALID_DATA;
		}
		else if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_READ && frame->lengthdir.u.length >= (16 + 4))
		{
			uint16_t nw_length = iziplus_networkframe_production_data_read(iziplus_nwdata, nw_data->seqid, (uint8_t *)PRODUCTION_DATA, sizeof(proddata_t));
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
		}
	}
#endif	
	else if(nw_data->cmd == IZIPLUS_CMD_SHUTDOWN && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT)
	{
		if(nw_data->msgtype.u.cmd_type == IZIPLUS_CMDTYPE_ACTION)
		{
			iziplus_cmd_shutdown_not_t *shutdown_not = (iziplus_cmd_shutdown_not_t *)&frame->data[sizeof(iziplus_network_data_t)];
			
			bool was_closed = StepDown_IsClosed();
			if(!was_closed)
			{
				StepDown_Close(1000);
				vTaskDelay(2);
				if(shutdown_not->time >= 100)		// Make sure there is enough time (no shutdown during store)
				{
					AppLog_Set();
#ifdef EMITTER_SUPPORT					
					AppEmitterLog_Set();			// Can take 100ms!		
#endif					
				}
			}
		}
	}
#if USE_CONTACT_TRIGGER
	else if(nw_data->cmd == IZIPLUS_CMD_CONTACTTRIGGER_ACK && nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT)			// Acknowledge contact triggers send
	{
		if(frame->lengthdir.u.length >= (sizeof(iziplus_network_data_t) + 1))//sizeof(iziplus_cmd_contacttriggerack_not_t)))
		{
			uint8_t ack_amount = (frame->lengthdir.u.length - sizeof(iziplus_network_data_t) / sizeof(contacttriggerack_t));
			iziplus_cmd_contacttriggerack_not_t *iziplus_cmd_contacttriggerack = (iziplus_cmd_contacttriggerack_not_t *)&frame->data[sizeof(iziplus_network_data_t)];
			for(int i = 0; i < ack_amount; i++)
			{
				if(iziplus_cmd_contacttriggerack->ack[i].short_id == iziplus_data_get_short_id())
				{
					if(iziplus_contact_event == iziplus_cmd_contacttriggerack->ack[i].event_id)			// received OK, stop sending
					{
						iziplus_contact_repeat = 0;					
						iziplus_contact_req = false;
						if(iziplus_contact_postponed >= 0)
							IziPlus_ContactReport((uint8_t)(iziplus_contact_postponed));
					}
				}
			}
		}
	}
#endif
	else if(nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_REQ)
		tokenresult = IZIPLUS_TOKERES_UNKNOWN_CMD;
	
	boot_data.last_code = 'F';
	if(tokenresult != IZIPLUS_TOKERES_OK)
	{
		uint16_t nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, nw_data->cmd, nw_data->msgtype.u.cmd_type, tokenresult);
		iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
	}
	else if(iziplus_cmd_delayed_req.state == DELAYED_RESPONSE)			// Delayed response ready?
	{
		uint16_t nw_length = 0;
		if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_FW_WRITE)			// Erase + prog ready?
		{
			if(iziplus_cmd_delayed_req.tokenresult == IZIPLUS_TOKERES_OK)	// Result OK?
				nw_length = iziplus_networkframe_firmwarewrite(iziplus_nwdata, nw_data->seqid);				// Send response
			else
				nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, IZIPLUS_CMD_FW_WRITE, nw_data->msgtype.u.cmd_type, iziplus_cmd_delayed_req.tokenresult);
		}
		else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_FW_APPLY)		// CRC check and start bootloader
		{
			if(iziplus_cmd_delayed_req.tokenresult == IZIPLUS_TOKERES_OK)	// Result OK?
			{
				nw_length = iziplus_networkframe_firmwareapply(iziplus_nwdata, nw_data->seqid);				// Send response
				iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);	
				State_SetAttentionInfo(COLOR_MAGENTA, COLOR_MAGENTA, 2, 1);
				StepDown_Close(1000);
				vTaskDelay(200);
				AppLog_Set();
#ifdef EMITTER_SUPPORT				
				AppEmitterLog_Set();										// Can take 80ms!		
#endif				
				
				boot_data.action = BOOT_ACTION_COPY_EXT;					// Reprogram
				boot_data.magic = BOOT_MAGIC_VALUE;
				NVIC_SystemReset();
			}
			else
				nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, IZIPLUS_CMD_FW_APPLY, nw_data->msgtype.u.cmd_type, iziplus_cmd_delayed_req.tokenresult);
		}
		else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_TEXT)			// Write text
		{
			if(iziplus_cmd_delayed_req.tokenresult == IZIPLUS_TOKERES_OK)	// Result OK?
				nw_length = iziplus_networkframe_textwrite(iziplus_nwdata, nw_data->seqid);				// Send response
			else
				nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, IZIPLUS_CMD_TEXT, nw_data->msgtype.u.cmd_type, iziplus_cmd_delayed_req.tokenresult);
		}
		else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_CONFIG)			// Write config
		{
			if(iziplus_cmd_delayed_req.tokenresult == IZIPLUS_TOKERES_OK)	// Result OK?
				nw_length = iziplus_networkframe_assign_configwrite(iziplus_nwdata, nw_data->seqid);				// Send response
			else
				nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, IZIPLUS_CMD_CONFIG, nw_data->msgtype.u.cmd_type, iziplus_cmd_delayed_req.tokenresult);
		}
		else if(iziplus_cmd_delayed_req.cmd == IZIPLUS_CMD_ASSIGN_CHANMODE)			// Write channel mode
		{
			if(iziplus_cmd_delayed_req.tokenresult == IZIPLUS_TOKERES_OK)	// Result OK?
				nw_length = iziplus_networkframe_assign_channelmodewrite(iziplus_nwdata, nw_data->seqid, IziPlus_Module_GetChannelAmount());
			else
				nw_length = iziplus_networkframe_error(iziplus_nwdata, nw_data->seqid, IZIPLUS_CMD_ASSIGN_CHANMODE, nw_data->msgtype.u.cmd_type, iziplus_cmd_delayed_req.tokenresult);
		}
		
		if(nw_length > 0)
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);	
		iziplus_cmd_delayed_req.state = DELAYED_EMPTY;
		iziplus_cmd_delayed_req.cmd = 0;
	}
	else if((nw_data->msgtype.u.frame_type == IZIPLUS_FRAMETYPE_NOT) && ((nw_data->tokenres == iziplus_data_get_short_id()) || (nw_data->tokenres == IZIPLUS_TOKEN_FREE)) && frame->pan_id != IZIPLUS_PANID_WILDCARD)		// Is it our token?
	{
		uint16_t nw_length;
		bool send_response = true;
#if USE_CONTACT_TRIGGER		
		if(iziplus_contact_repeat > 0)
		{
			if(nw_data->tokenres == iziplus_data_get_short_id())
			{
				nw_length = iziplus_networkframe_contactaction(iziplus_nwdata, nw_data->seqid, iziplus_contact_event, iziplus_contact_flags);
				iziplus_contact_repeat--;
			}
			else if(iziplus_contact_repeat > 0)
			{
				iziplus_contact_req = true;
				iziplus_contact_seqid = nw_data->seqid;
				iziplus_contact_pan_id = frame->pan_id;
				send_response = false;
			}
		}
		else if(iziplus_identify_local)
#else
		if(iziplus_identify_local)
#endif		
		{
			nw_length = iziplus_networkframe_identifyaction_not(iziplus_nwdata, nw_data->seqid);
			iziplus_identify_local = false;
		}
		else if (nw_data->tokenres == iziplus_data_get_short_id())			// Only report state if it was our token
		{
			if(iziplus_state_version_report)
				iziplus_state_version_report--;
				
			if(iziplus_state_version_report == 1)
			{
				izi_version_u protversion = { .v.major = IZIPLUS_PROTOCOL_VERSION_MAJOR, .v.minor = IZIPLUS_PROTOCOL_VERSION_MINOR };
				nw_length = iziplus_networkframe_versionread(iziplus_nwdata, nw_data->seqid, IziPlus_Module_GetAppVersion(), IziPlus_Module_GetBootVersion(), protversion, IziPlus_Module_GetTypedefVersion(),
#ifdef EMITTER_SUPPORT				
					IziPlus_Module_GetImageVersion(), IziPlus_Module_GetLedtableVersion(), IziPlus_Module_GetMemoryVersion(), IziPlus_Module_GetHwRevision(), IziPlus_Module_GetVariant(), true, emitter_fixdef.hwref, emitter_proddata.serial);
#else
					IziPlus_Module_GetImageVersion(), IziPlus_Module_GetLedtableVersion(), IziPlus_Module_GetMemoryVersion(), IziPlus_Module_GetHwRevision(), IziPlus_Module_GetVariant(), true, 0, 0);
#endif
			}
			else
				nw_length = iziplus_networkframe_stateread(iziplus_nwdata, nw_data->seqid, IZIPLUS_FRAMETYPE_NOT, IziPlus_Module_GetCpuState().w, IziPlus_Module_GetMonitor(), MONITOR_SIZE);
		}
		else
			send_response = false;
			
		if(send_response)
			iziplus_dataframe_response(frame->pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
#if USE_CONTACT_TRIGGER	
		else if(xTaskGetTickCount() > 5000 && StepDown_SleepCheck() && !iziplus_contact_req)
#else
		else if(xTaskGetTickCount() > 5000 && StepDown_SleepCheck())
#endif
		{
			iziplus_sleep_time = 10 + (256 - (frame->lengthdir.u.length - sizeof(iziplus_network_data_t))) / (256 / 8);			// Add extra sleep seconds when less bytes are received, max add 8 seconds
			//gpio_set_pin_level(LED_R, false);
			xSemaphoreGive(xIziPlus_DelaySemaphore);
		}
		if(nw_data->tokenres == iziplus_data_get_short_id())
			iziplus_com_tokens = IZIPLUS_COM_TOKEN_TIME / IZIPLUS_TIMEROS_TICKS;
	}
	else if(nw_data->tokenres == IZIPLUS_TOKEN_BLOCK)
	{
		/*if(iziplus_state == IZIPLUS_STATE_NORMAL && ((iziplus_com_reinit & 0x01) == 0) && iziplus_com_reinit < 8)
		{
			if(iziplus_active_time > ((iziplus_com_reinit * 150) + 150) && iziplus_data_comquality() <= 95)
				iziplus_com_reinit++;
		}*/
		if(iziplus_dataframe_loss() > 1)		// Bit weird, data loss is not reset, so it will be executed multiple times in this second
		{
			if(iziplus_active_time < 100)		// First 10 sec do every time
			{
				iziplus_com_reinit = 1;
			}
			else if(iziplus_active_time < 600)	// First minute do max once per 5 sec
			{
				if(++iziplus_com_reinit_cnt >= 50)
				{
					iziplus_com_reinit = 1;
					iziplus_com_reinit_cnt = 0;
				}
			}
			else								// After first minute do max once per 60 sec
			{
				if(++iziplus_com_reinit_cnt >= 600)
				{
					iziplus_com_reinit = 1;
					iziplus_com_reinit_cnt = 0;
				}
			}
		}
		else
		{
			iziplus_com_reinit_cnt /= 2;
			iziplus_com_reinit = 0;
		}
	}
	
	if(iziplus_state == IZIPLUS_STATE_COMMISSIONING)
		iziplus_state_com_timer = IZIPLUS_COMMISSIONING_EXTTIME / IZIPLUS_TIMEROS_TICKS;		// Command received in commissioning state? Extend time of commissioning state
	else
		iziplus_com_active_timer = IZIPLUS_COM_ACTIVE_TIME / IZIPLUS_TIMEROS_TICKS;
		
	boot_data.last_code = 'Z';
}

void IziPlus_HandleRxIdle()
{
	// Max delay 12ms!! Depending on he amount of masters this should be less, so it is probably better to use the 5ms which is best for 3 modules on the same frequency
#if USE_CONTACT_TRIGGER	
	if(iziplus_contact_req)
	{
		uint16_t nw_length;
		vTaskDelay(iziplus_contact_dly);
		
		iziplus_contact_req = false;
		nw_length = iziplus_networkframe_contactaction(iziplus_nwdata, iziplus_contact_seqid, iziplus_contact_event, iziplus_contact_flags);
		iziplus_contact_repeat--;
		if(iziplus_contact_repeat > 0)
		{
			iziplus_contact_dly = (iziplus_contact_dly + (rand_sync_read32(&RAND_0) & 0x06)) & 0x07;
		}
		else if(iziplus_contact_postponed >= 0)
		IziPlus_ContactReport((uint8_t)(iziplus_contact_postponed));
		
		iziplus_dataframe_response(iziplus_contact_pan_id, IZIPLUS_DEVTYPE, (uint8_t *)iziplus_nwdata, nw_length);
	}
#endif	
}

#if USE_CONTACT_TRIGGER
void IziPlus_ContactReport(uint8_t contact_flag)
{
	uint8_t vcontact = CONFIG_VIRTUAL_IN(appConfig->config);
	if(iziplus_contact_repeat == 0 && vcontact > 0)
	{
		iziplus_contact_repeat = IZIPLUS_CMD_CONTACTTRIGGER_REPEAT;		// Repeat max 12 times, if ACK is received
		iziplus_contact_flags = contact_flag;
		iziplus_contact_event = (iziplus_contact_event + 1) & 0x0F;		// Upper 4-bits are used for the type, in this case 0, is max 8 contacts
		iziplus_contact_dly = ((vcontact - 1) * 2) & 0x07;				// Offset of 0 2 4 6ms (sending contact trigger takes +/- 1.1 sec)
		iziplus_contact_postponed = -1;
	}
	else if(iziplus_contact_flags != contact_flag)
	{
		iziplus_contact_postponed = contact_flag;
	}
	else
	iziplus_contact_postponed = -1;
}
#endif

bool IziPlus_ComActive()
{
	return iziplus_com_active_timer > 0;
}

void IziPlus_IdentifyLocal()
{
	iziplus_identify_local = true;
}

void IziPlus_Powerup(uint8_t commissioning)
{
	if(commissioning)
	{
		State_SetStartInfo(STATE_START_COMMISSIONING);
		iziplus_pl_freq = commissioning > 1 ? IZIPLUS_DISCOVER_FREQ2 : IZIPLUS_DISCOVER_FREQ;
		iziplus_pl_rate = IZIPLUS_DISCOVER_RATE;
		iziplus_pl_txlevel = IZIPLUS_DISCOVER_TXLEVEL;
		
		iziplus_driver_ok = dcbm1_init(iziplus_pl_freq, iziplus_pl_rate, iziplus_pl_txlevel);
		dcbm1_enable_bus_busy(true);			// Check Bus busy during commissioning
	}
	else
	{
		State_ClearStartInfo(STATE_START_COMMISSIONING);
		iziplus_pl_freq = iziplus_data_get_frequency();
		iziplus_pl_rate = iziplus_data_get_rate();
		iziplus_pl_txlevel = iziplus_data_get_tx_level();				
		
		if(iziplus_driver_ok >= 0)
			iziplus_driver_ok = dcbm1_setcom(iziplus_pl_freq, iziplus_pl_rate, iziplus_pl_txlevel);
		else
			iziplus_driver_ok = dcbm1_init(iziplus_pl_freq, iziplus_pl_rate, iziplus_pl_txlevel);
			
		dcbm1_enable_bus_busy(false);			// Do not check Bus busy during commissioning
	}	
	
	State_ClearAttentionInfo();
	
	//izi_discover_req = IZIPLUS_REQ_SYNC;
	sprintf(text, "Power on -> Freq: %d.%dMHz Rate: %dkHz", dcbm1_getfrequency_h(iziplus_pl_freq), dcbm1_getfrequency_l(iziplus_pl_freq), dcbm1_getrate_kHz(iziplus_pl_rate));
	LOGWRITE_INFO(ERRCODE_IZI_POWERON_OPERATION, text, log_src_task_postpone);

	IZI_TRACE_INFO(text); GEN_TRACE_INFO("\r\n");
}

void IziPlus_SetCom()
{
	dcbm1_setcom(iziplus_pl_freq, iziplus_pl_rate, iziplus_pl_txlevel);
}


// Proceeded with 'I'
void IziPlus_Debug(uint8_t *data, uint8_t length)
{
	if(data[0] == 't' && length >= 2)
	{
		izi_trace_lvl = data[1] - '0';
		IZI_TRACE_ERROR("Trace level: %d\r\n", izi_trace_lvl);
	}
	
}

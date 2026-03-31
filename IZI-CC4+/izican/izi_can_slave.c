/*
 * izi_can_slave.c
 *
 *  Created on: 8 jul. 2020
 *      Author: Milo
 */

#include "includes.h"
#include "izi_can_slave.h"
#include "izi_can_frame.h"
#include "izi_can_driver.h"
#include "izi_can_slave_handler.h"
#include "izilink/izi_output.h"
#include "timers.h"
#include "appconfig.h"
#include "hpl_can.h"
#include "rtctime.h"
#include "version.h"
#include "firmware_info.h"
#include "izi_can_slave.h"
#include "logger_data.h"
#include "fifo.h"
#if IZIPLUS_DRIVER > 0
#include "izilink/iziplus_driver.h"
#endif
#if IZI_DRIVER
#include "izi_driver.h"
#endif
//#include "iziplus_driver.h"
#include "misc/state.h"
#include "misc/analog.h"

// Defines
#define IZICAN_SLAVE_HB_SEC		4			// Send every 4 sec a heartbeat
#define IZICAN_TIMEROS_TICKS	1000

#define IZICAN_DATA_TIMEOUT		6			// 6 seconds no data, report to + modules

#define TASK_IZICAN_SLAVE_STACK_SIZE (512)
#define TASK_IZICAN_SLAVE_TASK_PRIORITY (tskIDLE_PRIORITY + 3)

#define LOGGER_FIFO_MAX			16

#define POWER_REPORT_MINUTES	3
#define MONITOR_VALUES_LEN		8

#define _A __attribute__((aligned(4)))

// Globals
TimerHandle_t  	xIziCan_Timer = NULL;
static TaskHandle_t		xIziCanSlave_Task;
static uint16_t izican_slave_timer_prs;
static uint16_t izican_slave_channel_cnt, izican_slave_channel_total;
static uint16_t izican_slave_data_timeout = IZICAN_DATA_TIMEOUT;

static fifo_t izi_fifo_log_items;
static logitem_t log_items[LOGGER_FIFO_MAX];
static uint8_t log_item_idx = 0;
static _A uint8_t monitor_values[MONITOR_VALUES_LEN];

extern boot_t boot_data;

// Proto
void izican_slave_timer_sec(TimerHandle_t xTimer);
bool izican_slave_rxint_cb(struct can_message *frame);


/**
 * Called from izican task reporting received CAN frame (or multi frame), if not handled in interrupt cb
 */
void izican_slave_rx_cb(izi_can_rxframe_t *frame)
{
	izi_can_extid_u extid;
	extid.all = frame->id;

	if(extid.bit.dir != DIRECTION_TO_SLAVE)
	{
		IZI_CAN_TRACE_ERROR("Message to master received, weird!");
		return;
	}

	if(extid.bit.msg_type == IZI_CAN_MSGTYPE_NOT)
	{
		izican_slave_notification_handler(extid, (izican_notification_data_t *)frame->data, frame->length);
	}
	else if(extid.bit.msg_type == IZI_CAN_MSGTYPE_REQ)
	{
		TickType_t ticks2 = xTaskGetTickCount();	
		izican_slave_request_handler(extid, (izican_request_data_t *)frame->data, frame->length);
	
		TickType_t ticks = xTaskGetTickCount();	
		if(ticks - 25 > frame->ticks)
		{
			IZI_CAN_TRACE_ERROR("Message handling slow: %d ms (%d, %d ms)\r\n", ticks - frame->ticks, frame->ticks, ticks2 - frame->ticks);	
		}
	}
}

/*uint8_t last_data0;
uint8_t data0[256];
TickType_t last_tick;
TickType_t ticks[256];
uint16_t data0_idx = 0;
*/

volatile uint8_t last_sector0[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

/**
 * Called from interrupt, to save time and resources, copy here to output buffer
 */
bool izican_slave_rxint_cb(struct can_message *frame)
{
	if(frame->fmt == CAN_FMT_STDID)			// Channel frame?
	{
		izi_can_stdid_u izi_can_stdid;
		izi_can_stdid.all = frame->id;// >> STDID_OFFSET;

		uint16_t start = (izi_can_stdid.bit.start_channel * 64) + (izi_can_stdid.bit.universe * 512);
		Izi_OutputSetBuffer(start, frame->data, frame->len);
		if(izi_can_stdid.bit.sector0 > 0)
		{
			if(last_sector0[izi_can_stdid.bit.universe] != izi_can_stdid.bit.sector0)			// Save time not clearing the buffer every time
			{
				Izi_OutputSetBufferFixed(start + 64, 0, (izi_can_stdid.bit.sector0 * 64));			// Clear other 64 byte blocks that are set to 0
				last_sector0[izi_can_stdid.bit.universe] = izi_can_stdid.bit.sector0;
			}
		}
		else
			last_sector0[izi_can_stdid.bit.universe] = 0xFF;
			
		if(start == 0)
			izican_slave_channel_cnt++;
		izican_slave_channel_total++;
	
		izican_slave_data_timeout = IZICAN_DATA_TIMEOUT;
	
		/* Added to check for input
		if(start == 0)
		{
			data0[data0_idx] = frame->data[0];
			TickType_t tick = xTaskGetTickCountFromISR();
			ticks[data0_idx] = tick - last_tick;
			last_tick = tick;

			if(abs(last_data0 - frame->data[0]) > 3)
				data0_idx = 0;

			last_data0 = data0[data0_idx];

			if(++data0_idx >= sizeof(data0))
				data0_idx = 0;
		}
		else
		{
			data0_idx = 0;
		}*/
		return true;
	}
	return false;
}

uint8_t last_minute = 0xFF;

void izican_create_monitorvalues(uint8_t *bfr, uint8_t max_len)
{
	if(max_len >= 2)
		*((uint16_t *)&bfr[0]) = Analog_GetOutputCurrentMax();
	if(max_len >= 4)
		*((uint16_t *)&bfr[2]) = Analog_GetOutputVoltage();
	if(max_len >= 5)
		bfr[4] = appConfig->timing_offset;
}


/**
 * Timer from OS every sec
 */
void izican_slave_timer_sec(TimerHandle_t xTimer)
{
	if(++izican_slave_timer_prs >= IZICAN_SLAVE_HB_SEC)
	{
		izican_slave_timer_prs = 0;
		_A izican_not_slaveheartbeat_t hb;
		hb.serial_slave = PRODUCTION_SERIAL;
#if IZIPLUS_DRIVER > 0
		hb.module_cnt = IziPlus_ModuleAmount();
#elif IZI_DRIVER
		hb.module_cnt = izi_get_module_amount();
#else
		hb.module_cnt = 0;
#endif
		hb.state = can_slave_state;
		hb.app_version = firmare_info.version.all;
		hb.link_state = izican_slave_get_linkstate();
		hb.temperature = Analog_GetProcessorTemperature();
		hb.supply_voltage = Analog_GetSupplyVoltage();
		hb.output_current = Analog_GetOutputCurrent();
		izican_create_monitorvalues(hb.monitor, HB_MAX_MONITOR);
		hb.error_flags = State_GetErrors();
		hb.warning_flags = State_GetWarnings();
		IZI_CAN_TRACE_DEBUG("Send heart beat\r\n");
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_HEARTBEAT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&hb, sizeof(izican_not_slaveheartbeat_t), IZI_CAN_SEQ_AUTO);
	}
	izican_slave_update_state(true, izican_slave_timer_prs == 2);		// Report state changes (at least every 4 seconds)
	izican_create_monitorvalues(monitor_values, MONITOR_VALUES_LEN);
	izican_slave_update_monitor(monitor_values, MONITOR_VALUES_LEN, izican_slave_timer_prs == 3);		// Report state changes (at least every 4 seconds)
	
	uint32_t time = RtcTime_GetSeconds();
	uint8_t minute = (time % 3600) / (60/4);				// Send every 15 sec
	if(last_minute != minute)
	{
		uint8_t min3 = minute % (POWER_REPORT_MINUTES*4);	// Get modulo of per 15 sec
		Analog_PowerUpdate(min3, POWER_REPORT_MINUTES*4);
		izican_not_power_report_t pr;
		pr.amount = Analog_GetPower(pr.data);
		pr.minute = minute;
		pr.offset = POWER_REPORT_MINUTES;
		pr.total_energy = Analog_GetEnergy();
		last_minute = minute;
		izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_POWER_REPORT, IZI_CAN_MSGTYPE_NOT, (uint8_t *)&pr, POWER_REPORT_MIN_LENTGH + (pr.amount*sizeof(uint16_t)), IZI_CAN_SEQ_AUTO);
		
		//IZI_CAN_TRACE_INFO("Report Energy -> Energy: %d, Power: %d (%d), Amount: %d\r\n", pr.total_energy, pr.data[0], pr.data[pr.amount - 1], pr.amount);
		
		if(min3 == 0)
			Analog_PowerNext();		
	}
	
	if(izican_slave_channel_cnt > 0)
		IZI_CAN_TRACE_DEBUG("Received %d channel frames, start frames: %d\r\n", izican_slave_channel_total, izican_slave_channel_cnt);
	izican_slave_channel_cnt = 0;
	izican_slave_channel_total = 0;
	
	if(izican_slave_data_timeout > 0)
		--izican_slave_data_timeout;			
}

static bool master_present = false;
/************************************************************************/
/* IZI task														        */
/************************************************************************/
static void izican_slave_task(void *p)
{
	int prs = 0;
	State_SetStartInfo(STATE_START_CANBUS);

	IZI_CAN_TRACE_INFO("CAN slave start\r\n");
	
	while(1)
	{
		vTaskDelay(25);

		bool master_present_t = izican_master_present();
		if(master_present_t != master_present)
		{
			State_ClearStartInfo(STATE_START_CANBUS);
			IZI_CAN_TRACE_INFO("Master online: %d\r\n", master_present_t);
			master_present = master_present_t;
			if(master_present)
				State_ClearWarning(STATE_WARNING_NO_MASTER);
			else
				State_SetWarning(STATE_WARNING_NO_MASTER);
		}
		if(master_present)			// No need to update master when not present
		{
			if(++prs >= 40)					// Max every second
			{
				IziPlus_ReportConfig();
				IziPlus_ReportMonitor();
			}
			if( IziPlus_FastReportPending())
				IziPlus_ReportMonitor();
				
			static logitem_t logitem;
			if(fifo_get(izi_fifo_log_items, &logitem))
			{
				IZI_CAN_TRACE_INFO("Report log send: %d\r\n", logitem.errorcode);
				if(logitem.flags.severity >= log_warning)		// Only ask confirmation for errors and criticals
					izican_slave_notifylog(&logitem);
				else
					izican_slave_reportlog(&logitem);
			}
		}
		
		if(!izican_bus_ok())
			State_SetCan(CANSTATE_BUSOFF);
		else if(!master_present)
			State_SetCan(CANSTATE_NOMASTER);
		else if(can_slave_id < MAX_ADDRESS)
			State_SetCan(CANSTATE_READY);
		else 
			State_SetCan(CANSTATE_INIT);
			
		REPORT_STACK(TASK_IZICAN_SLAVE_STACK_SIZE, TASK_CANSLAVE);
	}
}

void izican_log_write(uint32_t errorcode, char *text, log_severity_t severity, uint32_t serial, log_source_t log_source)
{
	uint32_t time = RtcTime_GetSeconds();

	logitem_t *log_item = &log_items[log_item_idx];
	if(++log_item_idx >= LOGGER_FIFO_MAX)
		log_item_idx = 0;

//#ifdef FSL_RTOS_FREE_RTOS
    bool rtosRunning = xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
//#endif
	// 1598918400 = Tuesday, 1 September 2020 02:00:00 GMT+02:00
	log_item->flags.time_nosync = (time < 1598918400) ? 1 : 0;
	if(log_item->flags.time_nosync == 0)
		log_item->timestamp = time;
	else
		log_item->timestamp = rtosRunning ? xTaskGetTickCount() : 0;
	log_item->flags.severity = severity;
	log_item->flags.session = 0;
	log_item->errorcode = errorcode;
	strncpy(log_item->text, text, LOG_TEXT_SIZE);
	if(serial == 0)
		log_item->serial = PRODUCTION_SERIAL;
	else
		log_item->serial = serial;

	log_item->flags.log_source = log_source;

	TaskHandle_t task = xTaskGetCurrentTaskHandle();
	strncpy(log_item->task, pcTaskGetName(task), LOG_TASK_SIZE);

	fifo_add(izi_fifo_log_items, log_item);

	IZI_CAN_TRACE_INFO("Report log: %d\r\n", log_item_idx);
}

/**
 * Init CAN slave
 */
void izican_slave_init(void)
{
	izi_fifo_log_items = fifo_create(LOGGER_FIFO_MAX, sizeof(logitem_t));
	if (xTaskCreate(izican_slave_task, "IziCanSlave", TASK_IZICAN_SLAVE_STACK_SIZE, NULL, TASK_IZICAN_SLAVE_TASK_PRIORITY, &xIziCanSlave_Task) != pdPASS) {
		while (1) {
			;
		}
	}
	can_slave_id = appConfig->can_slave_id;

	if(boot_data.magic == BOOT_MAGIC_VALUE && boot_data.reset_cause != RESET_CAUSE_UNKNOWN && boot_data.reset_cause < IZICAN_STATE_RESET_UNKNOWN)
		can_slave_state = boot_data.reset_cause;
	else
		can_slave_state = IZICAN_STATE_RESET_UNKNOWN;

	if(can_slave_id > MAX_ADDRESS)
		can_slave_id = ADDRESS_WILDCARD;

	izican_driver_init(can_slave_id);
	izican_set_callback(izican_slave_rx_cb);
	izican_set_int_callback(izican_slave_rxint_cb);

	izican_slave_timer_prs = IZICAN_SLAVE_HB_SEC - 1;

	if(xIziCan_Timer == NULL)			// Init can be called multiple times
	{
		xIziCan_Timer = xTimerCreate("IzCanTimer", IZICAN_TIMEROS_TICKS, pdTRUE, ( void * ) 0, izican_slave_timer_sec);
		if(xIziCan_Timer == NULL)
		{
			IZI_CAN_TRACE_ERROR("Timer not created IziCan");				// Todo: what to do
		}
		else if( xTimerStart(xIziCan_Timer, 0) != pdPASS )
		{
			IZI_CAN_TRACE_ERROR("Timer not started IziCan");				// Todo: what to do
		}
	}
	else if( xTimerStart(xIziCan_Timer, 0) != pdPASS )
	{
		IZI_CAN_TRACE_ERROR("Timer not started IziCan");				// Todo: what to do
	}
}

#define IZICAN_DBG_BUFFER_SIZE	256
#define IZICAN_DBG_BUFFERS		16

static uint8_t izican_dbg_txbuffer[IZICAN_DBG_BUFFERS][IZICAN_DBG_BUFFER_SIZE];
static uint8_t izican_dbg_txbuffer_in = 0, izican_dbg_txbuffer_out = 0;
static volatile uint16_t izican_dbg_txbuffer_idx[IZICAN_DBG_BUFFERS];

void izican_dbg_init()
{
	izican_dbg_txbuffer_in = izican_dbg_txbuffer_out = 0;
	memset((uint8_t *)izican_dbg_txbuffer_idx, 0, sizeof(izican_dbg_txbuffer_idx));
}

void izican_slave_send()
{
	if(!master_present)
		return;
	
	for(int i = 0; i < 2; i++)
	{
		if(izican_dbg_txbuffer_idx[izican_dbg_txbuffer_out] > 0)
		{
			if(izican_dbg_txbuffer_in == izican_dbg_txbuffer_out && izican_dbg_txbuffer_idx[izican_dbg_txbuffer_out] > 0)	// All send?
			{
				if(++izican_dbg_txbuffer_in >= IZICAN_DBG_BUFFERS)	// Take a new input buffer and wait for izican_dbg_txbuffer_idx to be above 0
					izican_dbg_txbuffer_in = 0;
				izican_dbg_txbuffer_idx[izican_dbg_txbuffer_in] = 0;
			}
			
			if(i == 1)
				vTaskDelay(5);
			if(izican_sendcommand(can_slave_id, SLAVE_DEVTYPE_GROUP, IZI_CAN_CMD_DBG, IZI_CAN_MSGTYPE_NOT, (uint8_t *)izican_dbg_txbuffer[izican_dbg_txbuffer_out], izican_dbg_txbuffer_idx[izican_dbg_txbuffer_out], IZI_CAN_SEQ_AUTO) < IZI_CAN_OK)
				return;
			
			izican_dbg_txbuffer_idx[izican_dbg_txbuffer_out] = 0;
			if(++izican_dbg_txbuffer_out >= IZICAN_DBG_BUFFERS)
				izican_dbg_txbuffer_out = 0;
		}
		else
			break;
	}
}

void izican_slave_putc(char c)
{
	if(izican_dbg_txbuffer_idx[izican_dbg_txbuffer_in] >= IZICAN_DBG_BUFFER_SIZE)
	{
		if(++izican_dbg_txbuffer_in >= IZICAN_DBG_BUFFERS)
			izican_dbg_txbuffer_in = 0;
		izican_dbg_txbuffer_idx[izican_dbg_txbuffer_in] = 0;
		
		if(izican_dbg_txbuffer_in == izican_dbg_txbuffer_out)
			IZI_CAN_TRACE_ERROR("Debug overflow!!\r\n")
	}
	
	izican_dbg_txbuffer[izican_dbg_txbuffer_in][izican_dbg_txbuffer_idx[izican_dbg_txbuffer_in]] = c;
	izican_dbg_txbuffer_idx[izican_dbg_txbuffer_in]++;
}

bool izican_slave_getc(char *c)
{
	uint8_t bfr[2];
	int32_t rec = 0;//io_read(io, bfr, 1);
	if(rec > 0)
	{
		izican_slave_putc(bfr[0]);
		*c = bfr[0];
		return true;
	}
	return false;
}

bool izican_slave_nodata()
{
	return izican_slave_data_timeout == 0;
}
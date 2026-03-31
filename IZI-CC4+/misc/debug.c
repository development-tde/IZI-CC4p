/*
 * debug.c
 *
 *  Created on: 18 mei 2020
 *      Author: Milo
 */


#include "includes.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "izi_can_driver.h"
//#include "izi_driver.h"
#include "appconfig.h"
//#include "rtctime.h"
#include "version.h"
#include "dcbm1_driver.h"
#include "izi_output.h"
#include "misc/analog.h"
//#include "izi_driver.h"
//#include "iziplus_driver.h"
//#include "uart/usart2.h"
#include "mcp4725.h"
#include "dmx.h"
#include "usart2.h"
#include "iziplus_driver.h"

extern struct netif netif;

void Debug_Recv(uint8_t c);

#define DBG_MAX_BFR		128
uint8_t dbg_rx_bfr[DBG_MAX_BFR];
uint8_t dbg_rx_index;

#define DBG_MAX_HISTORY			32
#define DBG_MAX_HISTORY_BFRS	4
uint8_t dbg_history[DBG_MAX_HISTORY_BFRS][DBG_MAX_HISTORY] = {0};
uint8_t dbg_history_index = 0, dbg_history_cnt = 0;
uint8_t dbg_arrow_start = 0;
uint8_t dbg_on = false;
#define DBG_CAN_MASK		0x01
#define DBG_USB_MASK		0x02

#define DBG_MAX_RX_INT		64
volatile uint8_t dbg_rx_int[DBG_MAX_RX_INT], dbg_rx_int_idx;

#define TASK_DEBUG_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_DEBUG_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
static TaskHandle_t      xCreatedExampleTask;

SemaphoreHandle_t	rprintfSemaphore;
extern const uint8_t  mirrorflash;

// Proto
void Debug_RecvInt(uint8_t c);
void Debug_TxReadyInt();

static void Debug_Task(void *p)
{
	appconfig_t *cfgram = AppConfig_Get();
	dbg_on = cfgram->dbg_on;
	dbg_rx_int_idx = 0;
	
	Usart2_Init(Debug_TxReadyInt, Debug_RecvInt, NULL);
	Dmx_Direction(false);						// Turn the port to input again
	
	while(1)
	{
		vTaskDelay(100);
		for(int i = 0; i < dbg_rx_int_idx; i++)
		{
			Debug_Recv(dbg_rx_int[i]);
		}
		dbg_rx_int_idx = 0;
		//if(dbg_on & DBG_CAN_MASK)
		//	izican_slave_send();
		REPORT_STACK(TASK_DEBUG_STACK_SIZE, TASK_DEBUG);
	}
}

void Debug_TxReadyInt()
{
	Dmx_Direction(false);						// Turn the port to input again
}

void Debug_Init()
{
	dbg_rx_index = 0;
	rprintfSemaphore = xSemaphoreCreateMutex();
	
	if (xTaskCreate(Debug_Task, "Debug", TASK_DEBUG_STACK_SIZE, NULL, TASK_DEBUG_STACK_PRIORITY, xCreatedExampleTask)!= pdPASS) {
		while (1) {
			;
		}
	}
}

void Debug_On(uint8_t on)
{
	dbg_on = on;
}

void Debug_Putc(uint8_t c)
{
	if(dbg_on)
	{
		//if(dbg_on & DBG_CAN_MASK)
		//	izican_slave_putc(c);
		Dmx_Direction(true);						// Turn the port to output
		if(dbg_on & DBG_USB_MASK)
			Usart2_Putc(c);
	}
}

void Debug_RecvInt(uint8_t c)
{
	dbg_rx_int[dbg_rx_int_idx] = c;
	if(++dbg_rx_int_idx >= DBG_MAX_RX_INT)
		dbg_rx_int_idx = 0;
}

void Debug_Recv(uint8_t c)
{
	if (c== '\n')
		return;
	if(c >= 0x80)
		return;
	if(c == '\33')
	{
		dbg_arrow_start = 1;
		return;
	}
	if(c == '[' && dbg_arrow_start)
		return;

	dbg_rx_bfr[dbg_rx_index++] = c;
	if(dbg_rx_index >= DBG_MAX_BFR)
	{
		GEN_TRACE_INFO("Debug buffer overflow\r\n");
		dbg_rx_index = 0;
	}
	else
		dbg_rx_bfr[dbg_rx_index] = 0;

	if(c == '\r')
	{
		if(dbg_rx_index >= 3)
		{
			if(dbg_rx_bfr[0] == 'a')
			{
				GEN_TRACE_INFO("Supply Voltage: %d\r\n", Analog_GetSupplyVoltage());
				GEN_TRACE_INFO("Temperature   : %d\r\n", Analog_GetProcessorTemperature());
			}
#ifdef USART2_DMA	
			else if(dbg_rx_bfr[0] == 'Q')
			{
				Usart2_DmaTest();
			}
#endif
			else if(dbg_rx_bfr[0] == 'S')
			{
				StepDown_Debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}
			else if(dbg_rx_bfr[0] == 'V')
			{
				if(dbg_rx_bfr[1] == 'I')
					Mcp4725_Init();
				if(dbg_rx_bfr[1] == 'W')
					Mcp4725_WriteDac(dbg_rx_bfr[2] - '0', (dbg_rx_bfr[3] - '0') * 256);
			}
#ifdef FLASH_TEST
			else if(dbg_rx_bfr[0] == 'b')
			{
				int32_t status;
				uint8_t tst[256], cnt = 0;
				for(int i = 0; i < 256; i++)
					tst[i] = i;
				
				if(dbg_rx_bfr[1] == 'i')
				{
					for(int a = (FLASH_APP_MIRROR_START); a < 0x40000; a += 256)
					{
						TickType_t ticks = xTaskGetTickCount();
						if((status = flash_write_page_erase(&FLASH_0, a, (uint8_t *)tst, 256)) < ERR_NONE)		// Write to flash without erase
						{
							IZI_CAN_TRACE_INFO("Page Prog of 0x%08x failed: %d\r\n", a, status);
							break;
						}
						else
						{
							IZI_CAN_TRACE_INFO("Page Prog of 0x%08x OK: %d in %d ms\r\n", a, status, xTaskGetTickCount() - ticks);
						}
						if(++cnt >= 16)
						{
							vTaskDelay(100);
							cnt = 0;
						}
					}
				}
				else if(dbg_rx_bfr[1] == 'c')
				{
					if((status = flash_write_page_erase(&FLASH_0, 0x8000, (uint8_t *)tst, 256)) < ERR_NONE)		// Fuck up application flash
					{
						IZI_CAN_TRACE_INFO("Page Prog of 0x%08x failed: %d\r\n", 0x8000, status);
					}
					if((status = flash_write_page_erase(&FLASH_0, 0x2100, (uint8_t *)tst, 256)) < ERR_NONE)		// Fuck up application flash
					{
						IZI_CAN_TRACE_INFO("Page Prog of 0x%08x failed: %d\r\n", 0x2100, status);
					}
					else
					{
						IZI_CAN_TRACE_INFO("App code erase (partial) done\r\n");
					}
				}
			}
#endif			
			else if(dbg_rx_bfr[0] == 'c')
			{
				if(dbg_rx_bfr[1] == 'i')
					AppConfig_Init();
				else if(dbg_rx_bfr[1] == 'd')
					AppConfig_Default();
				else if(dbg_rx_bfr[1] == 'p')
					AppConfig_Print();
				else if(dbg_rx_bfr[1] == 's')
				{
					uint16_t i, nr = 0;
					for(i = 3; i < (dbg_rx_index - 1); i++)
					{
						nr *= 10;
						nr += dbg_rx_bfr[i] - '0';
					}
					appconfig_t *cfgram = AppConfig_Get();
					if(dbg_rx_bfr[2] == 's')
					{
						cfgram->serial = nr;
						sprintf(cfgram->name, "IZI-Link-PL-%04d", nr);
					}
					else if(dbg_rx_bfr[2] == 'f')
						cfgram->pl_frequency = nr;
					else if(dbg_rx_bfr[2] == 'r')
						cfgram->pl_rate = nr;

					AppConfig_Set();
				}
			}
			else if(dbg_rx_bfr[0] == 'o')
			{
				if(dbg_rx_bfr[1] == 'h')
				{
					size_t size = xPortGetFreeHeapSize();
					//size_t sizemin = xPortGetMinimumEverFreeHeapSize();
					OS_TRACE_INFO("Heap size:    %d of %d\r\n\r\n", (configTOTAL_HEAP_SIZE - size), configTOTAL_HEAP_SIZE);
				}
				/*else if(dbg_rx_bfr[1] == 's')
				{
					char tasks[40*16];
					vTaskList(tasks);
					OS_TRACE_INFO("Task list: Task State Prio Stack Num\r\nStates => B=Blocked, R=Ready, D=Deleted (waiting clean up), S=Suspended, or Blocked without a timeout\r\n%s", tasks);
				}*/
				else if(dbg_rx_bfr[1] == 'l')
				{
					os_report_stack = OS_REPORT_ALLTASKS;
					os_report_heap = true;
					OS_TRACE_INFO("Task log stack request\r\n");
				}
				//else if(dbg_rx_bfr[1] == 'c')
				//{
				//	OS_TRACE_INFO("Main clock: %d\r\n", CLOCK_GetCoreSysClkFreq());
				//}
			}
			//else if(dbg_rx_bfr[0] == 'i')
			//	izi_debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			else if(dbg_rx_bfr[0] == 'C')
			{
				//izican_debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}		
			else if(dbg_rx_bfr[0] == 'D')
			{
				OS_TRACE_INFO("Debug %c %d\r\n", dbg_rx_bfr[1] - '0', dbg_rx_bfr[2]);
				dbg_on = dbg_rx_bfr[1] - '0';			// Debug off (reset on new USB connection)
				if(dbg_rx_bfr[2] == 'S')
				{
					if(dbg_rx_bfr[3] == 'F')			// Store in flash
					{
						appconfig_t *cfgram = AppConfig_Get();
						cfgram->dbg_on = dbg_on;		
						AppConfig_Set();
						OS_TRACE_INFO("Debug on stored: %d\r\n", dbg_on);
					}
				}
			}	
			else if(dbg_rx_bfr[0] == 'I')
			{
				IziPlus_Debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}
			else if(dbg_rx_bfr[0] == 'P')
			{
				if(dbg_rx_bfr[1] == 'W')
					Production_Write(dbg_rx_bfr[2], "test", 'T', RtcTime_GetSeconds(), "Debug");
			}
#if 0			
			else if(dbg_rx_bfr[0] == 'r')
			{
				RtcTime_Debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}
#endif
			else if(dbg_rx_bfr[0] == 'd')
			{
				if(dbg_rx_bfr[1] == 'i')
					IziPlus_Powerup(false);
			}
#if IZI_DRIVER > 0
			//else if(dbg_rx_bfr[0] == 'i')
			//{
			//	izi_debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			//}
#endif
#if STATE_TRACE_BUILD_LVL >= 0
			else if(dbg_rx_bfr[0] == 's')
			{
				State_debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}
#endif
			else if(dbg_rx_bfr[0] == 'u')
			{
				Izi_Output_debug(&dbg_rx_bfr[1], dbg_rx_index - 2);
			}
			else if(dbg_rx_bfr[0] == 'w' && dbg_rx_bfr[1] == 'd' && dbg_rx_bfr[2] == 'r')
			{
				OS_TRACE_INFO("Wait for reset by WD\r\n");
				while(1);
			}
			else if(dbg_rx_bfr[0] == 'v')
			{
				if(dbg_rx_bfr[1] == 'a')
				{
					OS_TRACE_INFO("Version app: %d.%d.%d\r\n", firmare_info.version.v.major, firmare_info.version.v.minor, firmare_info.version.v.build);
				}
				else if(dbg_rx_bfr[1] == 'b')
				{
					version_t *boot_version = (version_t *)BOOT_VERSION_ADDRESS;
					OS_TRACE_INFO("Version boot: %d.%d.%d\r\n", boot_version->major, boot_version->minor, boot_version->build);
				}
			}
			else
				GEN_TRACE_INFO("\r\nUnknown cmd: %s", dbg_rx_bfr);
		}
		GEN_TRACE_INFO("\r\n> ");

		dbg_rx_index = 0;
		strncpy((char *)dbg_history[dbg_history_index++ & (DBG_MAX_HISTORY_BFRS - 1)], (char *)dbg_rx_bfr, DBG_MAX_HISTORY);
		dbg_history_cnt++;
	}
	else if(dbg_arrow_start && c == 'A')	// Up
	{
		if(dbg_history_cnt > 0)
		{
			strncpy((char *)dbg_rx_bfr, (char *)dbg_history[--dbg_history_index & (DBG_MAX_HISTORY_BFRS - 1)], DBG_MAX_HISTORY);
			dbg_rx_index = (uint8_t)(strlen((char *)dbg_rx_bfr) - 1);
			dbg_rx_bfr[dbg_rx_index] = 0;
			dbg_history_cnt--;
			GEN_TRACE_INFO("\r\n> %s", dbg_rx_bfr);
		}
		else
			dbg_rx_index = 0;
	}
	else if(c == 0x08)		// BS
	{
		if(dbg_rx_index > 0)
		{
			--dbg_rx_index;
			dbg_rx_bfr[--dbg_rx_index] = 0;
			GEN_TRACE_INFO("\n> %s", dbg_rx_bfr);
		}
	}
	dbg_arrow_start = 0;
}

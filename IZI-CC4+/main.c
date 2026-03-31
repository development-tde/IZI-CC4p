#include <atmel_start.h>
#include "includes.h"
#include "izi_can_driver.h"
#include "appconfig.h"
#include "firmware_info.h"
#include "version.h"
#include "misc/analog.h"
#include "izilink/izi_input.h"
#include "izi_output.h"
#include "smart_eeprom.h"
#include "izilink/iziplus_driver.h"
#include "stepdown.h"
#include "11LC160.h"
#if DMA_CHANNELS > 0	
#include "dmac.h"
#endif
#include "dmx.h"
#include "timing.h"
#include "bod.h"

boot_t __attribute__ ((section (".boot_ram"))) boot_data;
uint32_t os_report_stack;
bool os_report_heap;
static bool os_start = true;
static bool os_extra_log = false;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	// Handle stack overflow
	boot_data.last_code = '*';
	OS_TRACE_INFO("Stack overflow: %s\r\n", pcTaskName);
	for(;;);
}

void vApplicationIdleHook( void )
{
	TickType_t now = xTaskGetTickCount();
	boot_data.last_code = 'I';
	WD_Refresh();

	if(os_start && now > pdMS_TO_TICKS(5000))				// Running longer than 10 seconds, reset WD count
	{
		boot_data.reset_wd_cnt = 0;				// Todo: what to do when wd reset occurs too many times
		os_start = false;
		os_extra_log = true;
	}
	if(os_extra_log)
	{
		os_report_stack = OS_REPORT_ALLTASKS;
		os_report_heap = true;
		os_extra_log = false;
	}
#ifdef SLEEP_ON	
	/* Make sure we're not in deep sleep — deep sleep may stop peripheral clocks. */
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	/* Ensure all outstanding memory accesses complete before sleeping. */
	__DSB();
	__ISB();

	/* Wait For Interrupt - CPU clock stops, peripherals (GCLK/UART/Timers) continue. */
	__WFI();
#endif
	/* Execution resumes here immediately after any interrupt.  The core runs at full speed. */
}

static char text[LOG_TEXT_SIZE + 8];
uint8_t reset_cause;

void print_resetcause()
{
	if(boot_data.magic == BOOT_MAGIC_VALUE)
	{
		if(boot_data.reset_cause == RESET_CAUSE_POWER_UP)
		{
			sprintf(text, "Power-up, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_POWER_UP, text, log_src_task_postpone);
			OS_TRACE_INFO("Power-up, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		}
		else if(boot_data.reset_cause == RESET_CAUSE_WATCHDOG)
		{
			sprintf(text, "Watchdog reset, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_WATCHDOG, text, log_src_task_postpone);
			OS_TRACE_INFO("WD reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		}
		else if(boot_data.reset_cause == RESET_CAUSE_BOD)
		{
			sprintf(text, "BOD reset, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_BOD, text, log_src_task_postpone);
			OS_TRACE_INFO("BOD reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		}
		else if(boot_data.reset_cause == RESET_CAUSE_EXTRST)
		{
			sprintf(text, "Ext reset, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_EXTRST, text, log_src_task_postpone);
			OS_TRACE_INFO("Ext reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		}
		else if(boot_data.reset_cause == RESET_CAUSE_SYSRST)
		{
			sprintf(text, "Sys reset, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_SYSRST, text, log_src_task_postpone);
			OS_TRACE_INFO("Sys reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		}
		else
		{
			sprintf(text, "Sys reset, rst: %d, wd: %d", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
			LOGWRITE_WARNING(ERRCODE_RESET_UNKNOWN, text, log_src_task_postpone);
			OS_TRACE_INFO("Unknown reset cause\r\n");
		}
		reset_cause = boot_data.reset_cause;
	}
	else
	{
		sprintf(text, "Invalid reset struct");
		LOGWRITE_ERROR(ERRCODE_RESET_INVALID, text, log_src_task_postpone);
		OS_TRACE_INFO("No reset info (Reset stat: 0x%02X)\r\n", RSTC->RCAUSE.reg);

		reset_cause = RESET_CAUSE_SYSRST;			// Done for debugger
	}
	version_t *boot_version = (version_t *)BOOT_VERSION_ADDRESS;
	sprintf(text, "Version app: %d.%d.%d, boot: %d.%d.%d", firmare_info.version.v.major, firmare_info.version.v.minor, firmare_info.version.v.build, boot_version->major, boot_version->minor, boot_version->build);
	LOGWRITE_INFO(ERRCODE_VERSION_INFO, text, log_src_task_postpone);
	OS_TRACE_INFO("\r\nVersion app: %d.%d.%d, boot: %d.%d.%d\r\n", firmare_info.version.v.major, firmare_info.version.v.minor, firmare_info.version.v.build, boot_version->major, boot_version->minor, boot_version->build);
}

void ReportStackSize(uint32_t stackSize, uint8_t task_enum)
{
	if(os_report_heap)
	{
		size_t size = xPortGetFreeHeapSize();
		//size_t sizemin = xPortGetMinimumEverFreeHeapSize();

		OS_TRACE_INFO("Heap usage: %d percent of %dk\r\n", ((configTOTAL_HEAP_SIZE - size) * 100)/configTOTAL_HEAP_SIZE, configTOTAL_HEAP_SIZE / 1024/*, ((configTOTAL_HEAP_SIZE - sizemin) * 100)/configTOTAL_HEAP_SIZE*/);
		os_report_heap = false;
	}
	
	TaskStatus_t xTaskDetails;

	/* Use the handle to obtain further information about the task. */
	vTaskGetInfo( /* The handle of the task being queried. */
			  NULL,
			  /* The TaskStatus_t structure to complete with information
			  on xTask. */
			  &xTaskDetails,
			  /* Include the stack high water mark value in the
			  TaskStatus_t structure. */
			  pdTRUE,
			  /* Include the task state in the TaskStatus_t structure. */
			  eInvalid );

	uint16_t percentage = ((stackSize - xTaskDetails.usStackHighWaterMark) * 100L) / stackSize;
	OS_TRACE_INFO("Stack: %.10s %d percent (%d/%d)\r\n", xTaskDetails.pcTaskName, percentage, (stackSize - xTaskDetails.usStackHighWaterMark) *4, stackSize * 4);
}

int main(void)
{	
#ifdef DEBUG	
	boot_data.magic = BOOT_MAGIC_VALUE;
	boot_data.reset_cause = RESET_CAUSE_POWER_UP;
#endif

	boot_data.last_code_copy = boot_data.last_code;
	boot_data.last_data_copy = boot_data.last_data;
	boot_data.last_data2_copy = boot_data.last_data2;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	if(Eeprom_init())			// If fuses are set, restart
	{
#ifdef BOD_ENABLED		
		bod33_ensure(BOD33_LEVEL_FROM_VOLTAGE(2.8f), BOD33_ACTION_RESET, BOD33_HYST_ON);
#endif
		NVIC_SystemReset();
	}
#ifdef BOD_ENABLED	
	else if(bod33_ensure(BOD33_LEVEL_FROM_VOLTAGE(2.8f), BOD33_ACTION_RESET, BOD33_HYST_ON))
		NVIC_SystemReset();
#endif
		
	IziPlus_Module_SetFixtureParameters();
	AppConfig_Init();

	if(PRODUCTION_SERIAL == 0xFFFFFFFF)
	{
		Production_Write((rand_sync_read32(&RAND_0) & 0xFFFF) + 0x0BCD0000, "Auto gen.", 'T', 0, "-");
	}
#if DMA_CHANNELS > 0	
	DMA_Init();
#endif
	
	timing_init();
//	Debug_Init();

	State_Init();
	IziInput_Init();
	IziPlus_Init();
	StepDown_Init();
	//print_resetcause();
	
	vTaskStartScheduler();
	
	while (1) {
		// Should not get here
	}
}

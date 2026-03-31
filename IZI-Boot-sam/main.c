#include <atmel_start.h>
#include "includes.h"
#include "crc.h"
#include "version.h"

boot_t __attribute__ ((section (".boot_ram"))) boot_data;


void print_resetcause()
{
	if(hri_rstc_get_RCAUSE_POR_bit(RSTC))
	{
		//OS_TRACE_INFO("Power-up, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		boot_data.reset_cause = RESET_CAUSE_POWER_UP;
	}
	else if(hri_rstc_get_RCAUSE_WDT_bit(RSTC))
	{
		//OS_TRACE_INFO("WD reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		boot_data.reset_wd_cnt++;
		boot_data.reset_cause = RESET_CAUSE_WATCHDOG;
	}
	else if(hri_rstc_get_RCAUSE_BODCORE_bit(RSTC) || hri_rstc_get_RCAUSE_BODVDD_bit(RSTC)) 
	{
		//OS_TRACE_INFO("BOD reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		boot_data.reset_cause = RESET_CAUSE_BOD;
	}
	else if(hri_rstc_get_RCAUSE_EXT_bit(RSTC))
	{
		//OS_TRACE_INFO("Ext reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		boot_data.reset_cause = RESET_CAUSE_EXTRST;
	}
	else if(hri_rstc_get_RCAUSE_SYST_bit(RSTC))
	{
		//OS_TRACE_INFO("Sys reset, resets: %d, wd: %d\r\n", (int)boot_data.reset_cnt, (int)boot_data.reset_wd_cnt);
		boot_data.reset_cause = RESET_CAUSE_SYSRST;
	}
	else
	{
		//OS_TRACE_INFO("Unknown reset cause\r\n");
		boot_data.reset_cause = RESET_CAUSE_UNKNOWN;
	}
	 boot_data.reset_cnt++;
}

void JumpToApplication (void)
{
	uint32_t APP_START_ADDRESS = FLASH_APP_START;
	uint32_t app_start_address2;

	app_start_address2 = *(uint32_t *)(APP_START_ADDRESS +4);
	/* Re-base the Stack Pointer */
	__set_MSP(*(uint32_t *)APP_START_ADDRESS);

	/* Re-base the vector table base address */
	SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

	__disable_irq();
	
	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address2));
}

#define LED_R GPIO(GPIO_PORTB, 1)
#define LED_G GPIO(GPIO_PORTB, 2)
#define LED_B GPIO(GPIO_PORTB, 0)

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void WhiteLight(void)
{
    gpio_set_pin_direction(LED_R, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_R, false);
    gpio_set_pin_function(LED_R, GPIO_PIN_FUNCTION_OFF);
    
    gpio_set_pin_direction(LED_G, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_G, false);
    gpio_set_pin_function(LED_G, GPIO_PIN_FUNCTION_OFF);
    
    gpio_set_pin_direction(LED_B, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_B, false);
	gpio_set_pin_function(LED_B, GPIO_PIN_FUNCTION_OFF);
}

void WD_Refresh()
{
	CRITICAL_SECTION_ENTER();
	
	if(!WDT->SYNCBUSY.reg)
	WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
	CRITICAL_SECTION_LEAVE();
	//wdt_feed(&WDT_0);			this routine takes very long because it waits for a sync of something which takes more than 2ms, super weird...
}

int main(void)
{
	if(boot_data.action == BOOT_ACTION_STARTAPP && boot_data.magic == BOOT_MAGIC_VALUE)
	{
		boot_data.action = BOOT_ACTION_NONE;
		JumpToApplication();
	}
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	if(boot_data.magic != BOOT_MAGIC_VALUE)
	{
		boot_data.action = BOOT_ACTION_NONE;
		boot_data.reset_cause = RESET_CAUSE_UNKNOWN;
		boot_data.reset_cnt = 0;
		boot_data.reset_wd_cnt = 0;
		boot_data.magic = BOOT_MAGIC_VALUE;
	}

	if(version.all != 0xFFFFFFFF)			// Force version is included and not optimized
		print_resetcause();

	// Red LEDs
	gpio_set_pin_direction(LED_R, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_R, false);
	gpio_set_pin_function(LED_R, GPIO_PIN_FUNCTION_OFF);
	
	gpio_set_pin_direction(LED_G, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_G, true);
	gpio_set_pin_function(LED_G, GPIO_PIN_FUNCTION_OFF);
	
	gpio_set_pin_direction(LED_B, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_B, true);
	gpio_set_pin_function(LED_B, GPIO_PIN_FUNCTION_OFF);

	
	bool app_ok = false, mirror_ok = false;
	firmware_t *fw_info = (firmware_t *)(FLASH_APP_END - FLASH_APP_FW_INFO_SIZE);
	if(fw_info->image.start < (FLASH_APP_MIRROR_BASE) && fw_info->image.end < (FLASH_APP_MIRROR_BASE) && fw_info->image.start < fw_info->image.end)
	{
		uint16_t crc = 0x0;
		crc = Crc16Fast((uint8_t *)(fw_info->image.start), (fw_info->image.end + 1) - fw_info->image.start, crc);

		OS_TRACE_INFO("Local CRC: 0x%04x <--> 0x%04x (start: 0x%08x, end: 0x%08x)\r\n", crc, fw_info->image.crc, fw_info->image.start, fw_info->image.end);
		if (crc == fw_info->image.crc)					// Can the app be started?
		{
			app_ok = true;
		}
	}
	
	firmware_t *fw_info_mirror = (firmware_t *)(FLASH_APP_MIRROR_END - FLASH_APP_FW_INFO_SIZE);
	if(fw_info_mirror->image.start != 0xFFFFFFFF && fw_info_mirror->image.start < (FLASH_APP_MIRROR_BASE) && fw_info_mirror->image.end < (FLASH_APP_MIRROR_BASE) && fw_info_mirror->image.start < fw_info_mirror->image.end)
	{
		uint16_t crc = 0x0;
		crc = Crc16Fast((uint8_t *)(fw_info_mirror->image.start + (FLASH_APP_MIRROR_BASE - FLASH_APP_MIRROR_SHIFT)), (fw_info_mirror->image.end + 1) - fw_info_mirror->image.start, crc);

		OS_TRACE_INFO("Mirror CRC: 0x%04x <--> 0x%04x (start: 0x%08x, end: 0x%08x)\r\n", crc, fw_info_mirror->image.crc, fw_info_mirror->image.start, fw_info_mirror->image.end);
		if (crc == fw_info_mirror->image.crc)					// Can the app be started?
		{
			mirror_ok = true;
		}		
	}
		
	OS_TRACE_INFO("App: %d, Mirror: %d, Action: %x, Magic: %x\r\n", app_ok, mirror_ok, boot_data.action, boot_data.magic);
	//USART4_Send();
	
	bool reprogram = false;
	if(app_ok && mirror_ok)
	{
		if(boot_data.magic == BOOT_MAGIC_VALUE && boot_data.action == BOOT_ACTION_COPY_EXT)
			reprogram = true;
	}
	else if(mirror_ok)
		reprogram = true;
		
	if(reprogram)
	{
		gpio_set_pin_direction(LED_G, GPIO_DIRECTION_OUT);
		gpio_set_pin_level(LED_G, true);
		gpio_set_pin_function(LED_G, GPIO_PIN_FUNCTION_OFF);
		
		// Copy
		OS_TRACE_INFO("Copy mirror to app\r\n");
		//USART4_Send();
		for(int a = FLASH_APP_START; a < FLASH_APP_END; a += NVMCTRL_PAGE_SIZE)
		{
			flash_write_page_erase(&FLASH_0, a, (uint8_t *)(a + FLASH_APP_MIRROR_BASE - FLASH_APP_MIRROR_SHIFT), NVMCTRL_PAGE_SIZE);
			WD_Refresh();
		}
		boot_data.action = BOOT_ACTION_NONE;
		NVIC_SystemReset();
	}
	
	if(app_ok)
	{
		OS_TRACE_INFO("Starting application...\r\n");
		//USART4_Send();
		boot_data.action = BOOT_ACTION_STARTAPP;
		NVIC_SystemReset();
	}
	
	gpio_set_pin_direction(LED_B, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LED_B, false);
	gpio_set_pin_function(LED_B, GPIO_PIN_FUNCTION_OFF);
	
	while(1);
}

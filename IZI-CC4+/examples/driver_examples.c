/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"
#include "firmware_info.h"
static uint8_t src_data[512];
static uint8_t chk_data[512];

extern struct flash_descriptor FLASH_0;

//const uint8_t  __attribute__ ((section (".appsettingsection"))) appsettings2;

/**
 * Example of using FLASH_0 to read and write Flash main array.
 */
void FLASH_0_example(void)
{
	uint32_t page_size;
	uint16_t i;

	/* Init source data */
	page_size = flash_get_page_size(&FLASH_0);

	for (i = 0; i < page_size; i++) {
		src_data[i] = i;
	}
	
	uint32_t address = FLASH_APP_APPCFG_START;

	/* Write data to flash */
	flash_write(&FLASH_0, address, src_data, page_size);

	/* Read data from flash */
	flash_read(&FLASH_0, address, chk_data, page_size);
}



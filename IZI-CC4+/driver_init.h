/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "includes.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>
#include <hal_flash.h>
#include <hal_wdt.h>
#include <hal_can_async.h>

extern struct wdt_descriptor       WDT_0;
extern struct flash_descriptor	   FLASH_0;
extern struct rand_sync_desc		RAND_0;

void WDT_0_CLOCK_init(void);
void WDT_0_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);
uint8_t get_hw_id();
uint8_t get_device_id();

void WD_Refresh();

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED

/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "includes.h"
#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hal_rand_sync.h>
#include "uart/usart4.h"
#include "hal_usb_device.h"

struct wdt_descriptor WDT_0;
struct flash_descriptor FLASH_0;
struct rand_sync_desc RAND_0;

void FLASH_0_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_NVMCTRL_bit(MCLK);
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

void WDT_0_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
}

void WDT_0_init(void)
{
	WDT_0_CLOCK_init();
	wdt_init(&WDT_0, WDT);
}

void WD_Init()
{
	uint32_t clk_rate;
	uint16_t timeout_period;

	WDT_0_init();
	
	clk_rate       = 1000;
	timeout_period = 2048;
	wdt_set_timeout_period(&WDT_0, clk_rate, timeout_period);
	wdt_enable(&WDT_0);
}

void WD_Refresh()
{
	CRITICAL_SECTION_ENTER();
	
	if(!WDT->SYNCBUSY.reg)
		WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
	CRITICAL_SECTION_LEAVE();
	//wdt_feed(&WDT_0);			this routine takes very long because it waits for a sync of something which takes more than 2ms, super weird...
}

#if 0
void USB_DEVICE_INSTANCE_PORT_init(void)
{
	gpio_set_pin_direction(PA24,
	// <y> Pin direction
	// <id> pad_direction
	// <GPIO_DIRECTION_OFF"> Off
	// <GPIO_DIRECTION_IN"> In
	// <GPIO_DIRECTION_OUT"> Out
	GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA24,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	gpio_set_pin_pull_mode(PA24,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA24,
	// <y> Pin function
	// <id> pad_function
	// <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	// <PINMUX_PA24H_USB_DM"> Auto
	// <GPIO_PIN_FUNCTION_OFF"> Off
	// <GPIO_PIN_FUNCTION_A"> A
	// <GPIO_PIN_FUNCTION_B"> B
	// <GPIO_PIN_FUNCTION_C"> C
	// <GPIO_PIN_FUNCTION_D"> D
	// <GPIO_PIN_FUNCTION_E"> E
	// <GPIO_PIN_FUNCTION_F"> F
	// <GPIO_PIN_FUNCTION_G"> G
	// <GPIO_PIN_FUNCTION_H"> H
	// <GPIO_PIN_FUNCTION_I"> I
	// <GPIO_PIN_FUNCTION_J"> J
	// <GPIO_PIN_FUNCTION_K"> K
	// <GPIO_PIN_FUNCTION_L"> L
	// <GPIO_PIN_FUNCTION_M"> M
	// <GPIO_PIN_FUNCTION_N"> N
	PINMUX_PA24H_USB_DM);

	gpio_set_pin_direction(PA25,
	// <y> Pin direction
	// <id> pad_direction
	// <GPIO_DIRECTION_OFF"> Off
	// <GPIO_DIRECTION_IN"> In
	// <GPIO_DIRECTION_OUT"> Out
	GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA25,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	gpio_set_pin_pull_mode(PA25,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA25,
	// <y> Pin function
	// <id> pad_function
	// <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	// <PINMUX_PA25H_USB_DP"> Auto
	// <GPIO_PIN_FUNCTION_OFF"> Off
	// <GPIO_PIN_FUNCTION_A"> A
	// <GPIO_PIN_FUNCTION_B"> B
	// <GPIO_PIN_FUNCTION_C"> C
	// <GPIO_PIN_FUNCTION_D"> D
	// <GPIO_PIN_FUNCTION_E"> E
	// <GPIO_PIN_FUNCTION_F"> F
	// <GPIO_PIN_FUNCTION_G"> G
	// <GPIO_PIN_FUNCTION_H"> H
	// <GPIO_PIN_FUNCTION_I"> I
	// <GPIO_PIN_FUNCTION_J"> J
	// <GPIO_PIN_FUNCTION_K"> K
	// <GPIO_PIN_FUNCTION_L"> L
	// <GPIO_PIN_FUNCTION_M"> M
	// <GPIO_PIN_FUNCTION_N"> N
	PINMUX_PA25H_USB_DP);
	
	// Set pin direction to input
	gpio_set_pin_direction(USB_VBUS, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(USB_VBUS,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(USB_VBUS, GPIO_PIN_FUNCTION_OFF);
}


/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_DEVICE_INSTANCE_CLOCK_init(void)
{

	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);
}

void USB_DEVICE_INSTANCE_init(void)
{
	USB_DEVICE_INSTANCE_CLOCK_init();
	usb_d_init();
	USB_DEVICE_INSTANCE_PORT_init();
}

#endif

void RAND_0_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_TRNG_bit(MCLK);
}

void RAND_0_init(void)
{
	RAND_0_CLOCK_init();
	rand_sync_init(&RAND_0, TRNG);
	rand_sync_enable(&RAND_0);
}

uint8_t get_hw_id()
{
	uint8_t hw_id = 0;
	
	hw_id |= gpio_get_pin_level(HW_ID0) ? 0x01 : 0x00;
	hw_id |= gpio_get_pin_level(HW_ID1) ? 0x02 : 0x00;
	hw_id |= gpio_get_pin_level(HW_ID2) ? 0x04 : 0x00;
	hw_id |= gpio_get_pin_level(HW_ID3) ? 0x08 : 0x00;
	
	if(hw_id == 0x0F)
		return 1;
	return hw_id + 1;
}

uint8_t get_device_id()
{
	uint8_t hw_id = 0;
	
	hw_id |= gpio_get_pin_level(DEV_ID0) ? 0x01 : 0x00;
	hw_id |= gpio_get_pin_level(DEV_ID1) ? 0x02 : 0x00;
	hw_id |= gpio_get_pin_level(DEV_ID2) ? 0x04 : 0x00;
	//hw_id |= gpio_get_pin_level(DEV_ID3) ? 0x08 : 0x00;
	
	return hw_id;
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA00

	// Set pin direction to input
	gpio_set_pin_direction(HW_ID0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(HW_ID0,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(HW_ID0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA01

	// Set pin direction to input
	gpio_set_pin_direction(HW_ID1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(HW_ID1,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(HW_ID1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA02

	// Set pin direction to input
	gpio_set_pin_direction(HW_ID2, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(HW_ID2,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(HW_ID2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA03

	// Set pin direction to input
	gpio_set_pin_direction(HW_ID3, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(HW_ID3,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(HW_ID3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA04

	// Set pin direction to input
	gpio_set_pin_direction(DEV_ID0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DEV_ID0,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(DEV_ID0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA05

	// Set pin direction to input
	gpio_set_pin_direction(DEV_ID1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DEV_ID1,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(DEV_ID1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	// Set pin direction to input
	gpio_set_pin_direction(DEV_ID2, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DEV_ID2,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(DEV_ID2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	// Set pin direction to input
	/*gpio_set_pin_direction(DEV_ID3, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DEV_ID3,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(DEV_ID3, GPIO_PIN_FUNCTION_OFF);*/

	// GPIO on PA15

	gpio_set_pin_level(PLC_RST,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	// Set pin direction to output
	gpio_set_pin_direction(PLC_RST, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PLC_RST, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA16

	gpio_set_pin_level(PLC_CSEL1,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	// Set pin direction to output
	gpio_set_pin_direction(PLC_CSEL1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PLC_CSEL1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18

	gpio_set_pin_level(PLC_HDC,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	// Set pin direction to output
	gpio_set_pin_direction(PLC_HDC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PLC_HDC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	// Set pin direction to input
	/*gpio_set_pin_direction(PLC_RTR, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PLC_RTR,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PLC_RTR, GPIO_PIN_FUNCTION_OFF);
*/
	// GPIO on PA20

	// Set pin direction to input
	gpio_set_pin_direction(PLC_TXON, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PLC_TXON,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PLC_TXON, GPIO_PIN_FUNCTION_OFF);

	
	// Set pin direction to input
	gpio_set_pin_direction(SWITCH, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SWITCH,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_UP);

	gpio_set_pin_function(SWITCH, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_level(PLC_CSEL0,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	// Set pin direction to output
	gpio_set_pin_direction(PLC_CSEL0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PLC_CSEL0, GPIO_PIN_FUNCTION_OFF);
	
	
	// Set pin direction to output
	gpio_set_pin_direction(EXT, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(EXT, GPIO_PULL_UP);
	gpio_set_pin_function(EXT, GPIO_PIN_FUNCTION_OFF);
	
	
//	gpio_set_pin_function(PA20, PINMUX_PA20A_EIC_EXTINT4);
	
	
	// Ext int
	/*hri_gclk_write_PCHCTRL_reg(GCLK, EIC_GCLK_ID, CONF_GCLK_EIC_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_EIC_bit(MCLK);
	
	EIC->CTRLA.bit.SWRST = 1;
	while ((EIC->SYNCBUSY.reg) & EIC_SYNCBUSY_SWRST) {
	};
	
	EIC->CONFIG[1].reg = EIC_CONFIG_SENSE6_FALL;
	EIC->CONFIG[0].reg = EIC_CONFIG_SENSE4_FALL;
	EIC->INTENSET.reg = (1 << 4) | (1 << 14);
	
	gpio_set_pin_function(OUT_OC, PINMUX_PB14A_EIC_EXTINT14);
	gpio_set_pin_function(PA20, PINMUX_PA20A_EIC_EXTINT4);
	
	NVIC_DisableIRQ(EIC_4_IRQn);
	NVIC_ClearPendingIRQ(EIC_4_IRQn);
	NVIC_SetPriority(EIC_4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(EIC_4_IRQn);
	
	NVIC_DisableIRQ(EIC_14_IRQn);
	NVIC_ClearPendingIRQ(EIC_14_IRQn);
	NVIC_SetPriority(EIC_14_IRQn, 0);
	NVIC_EnableIRQ(EIC_14_IRQn);
	
	EIC->CTRLA.bit.ENABLE = 1;
	
	// Check CONF_EIC_ENABLE_IRQ_SETTINGxx, CONF_EIC_SENSExx, CONFIG_EIC_EXTINT_MAP and EXT_IRQ_AMOUNT if interrupts are added
	//ext_irq_init();
		
	//USB_DEVICE_INSTANCE_init();
	*/
#ifndef DEBUG 	
	WD_Init();
#endif	
	FLASH_0_init();
	//USART4_Init();
	RAND_0_init();
}

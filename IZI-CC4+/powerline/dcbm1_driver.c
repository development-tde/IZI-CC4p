/*
 * dcbm1_driver.c
 *
 *  Created on: 4 sep. 2020
 *      Author: Milo
 */

#include "includes.h"
#include "dcbm1_driver.h"
#include "appconfig.h"
//#include "hal_ext_irq.h"
#ifndef DCBM1_UART_INCLUDE
#include "uart/usart1.h"
#else 
#include DCBM1_UART_INCLUDE
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/
#ifndef DCBM1_UART_INIT
#define DCBM1_UART_INIT					Usart1_Init
#endif //DCBM1_UART_INIT
#ifndef DCBM1_UART_PUTC
#define DCBM1_UART_PUTC					Usart1_Putc
#endif //DCBM1_UART_PUTC
#ifndef DCBM1_UART_PUTBFR
#define DCBM1_UART_PUTBFR				Usart1_PutBfr
#endif //DCBM1_UART_PUTBFR

#define DCBM1_HDC(level)				gpio_set_pin_level(PLC_HDC, level)  
#define DCBM1_RST(level)				gpio_set_pin_level(PLC_RST, level)	
#define DCBM1_CSEL0(level)				gpio_set_pin_level(PLC_CSEL0, level)
#define DCBM1_CSEL1(level)				gpio_set_pin_level(PLC_CSEL1, level)
#define DCBM1_EN_TX(level)				DCBM1_CSEL0(level)

#define DCBM1_CSEL0_DIR(dir)			gpio_set_pin_direction(PLC_CSEL0, dir);
#define DCBM1_CSEL1_DIR(dir)			gpio_set_pin_direction(PLC_CSEL1, dir);
#define DCBM1_CSEL1_BUS_BUSY			gpio_get_pin_level(PLC_CSEL1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
bool dcbm1_writereg(uint8_t reg, uint8_t data);
bool dcbm1_writesync();
bool dcbm1_readreg(uint8_t reg, uint8_t *data);


/*******************************************************************************
 * Variables
 ******************************************************************************/
#define DCBM1_MAX_TX_BUFFER	1024
static volatile uint8_t tx_busy;

static volatile char rx_data;
static volatile bool rx_rec = false;
static volatile uint8_t rx_commandmode = 0;
static volatile uint8_t tx_loop = 0;
static volatile uint8_t is_sleeping = false;

static volatile uint8_t if_pin_func = 0;		// Interface pin functionality

trace_level_t dcbm1_trace_lvl = DCBM1_DFLT_TRACE_LVL;

static SemaphoreHandle_t xDcbm1_RxSemaphore = NULL;
static SemaphoreHandle_t xDcbm1_TxSemaphore = NULL;
static dcbm1_cb_func_rxptr dcbm1_cb_func_rx = NULL;
static dcbm1_cb_func_txptr dcbm1_cb_func_tx = NULL;
static volatile int dcbm1_extint_cnt, dcbm1_extint_dly;
static volatile bool dcbm1_delayed_tx = false;
static volatile bool dcbm1_bus_busy_enable = false;

extern boot_t boot_data;

static void dcbm1_tx_cb()
{
	if(tx_busy)
	{
		boot_data.last_code = 'Y';
		if(if_pin_func == 0x01)
		{
			if(!rx_commandmode)
			{
				if(!dcbm1_bus_busy_enable || (!(DCBM1_CSEL1_BUS_BUSY)))
					DCBM1_EN_TX(1);			// Indicate it can be send
				else
					dcbm1_delayed_tx = true;
			}
			xSemaphoreGiveFromISR(xDcbm1_TxSemaphore, NULL);
			portEND_SWITCHING_ISR(true);		// Make sure the semaphore is directly handled (iso the next tick)
			
			if(!rx_commandmode)
			{
				DCBM1_EN_TX(0);
			}
		}
		tx_busy = 0;
		boot_data.last_code = '#';
	}
}

static void dcbm1_err_cb()
{
	dcbm1_extint_cnt++;
}

static void dcbm1_rx_cb(uint8_t c)
{
	boot_data.last_code = 'Z';
	if(!rx_commandmode)
	{
		if(dcbm1_cb_func_rx != NULL)
			dcbm1_cb_func_rx(c);
		return;
	}
		
	rx_data = c;
	rx_rec = true;
		
	xSemaphoreGiveFromISR(xDcbm1_RxSemaphore, NULL);
	portEND_SWITCHING_ISR(true);		// Make sure the semaphore is directly handled (iso the next tick)
	
	boot_data.last_code = 'X';
}

void dcbm1_usart_init(void)
{
	DCBM1_UART_INIT(dcbm1_tx_cb, dcbm1_rx_cb, dcbm1_err_cb);
}

void dcbm1_set_rx_callback(dcbm1_cb_func_rxptr ptr)
{
	dcbm1_cb_func_rx = ptr;
	DCBM1_UART_INIT(dcbm1_tx_cb, ptr, dcbm1_err_cb);			// Make rx ptr direct call
}

void dcbm1_set_txready_callback(dcbm1_cb_func_txptr ptr)
{
	dcbm1_cb_func_tx = ptr;
}

#ifdef DCBM1_TRACE_ENABLE
// Command is 'd'
void dcbm1_debug(uint8_t *data, uint8_t length)
{
	if(data[0] == 't' && length >= 2)
	{
		dcbm1_trace_lvl = data[1] - '0';
		DCBM1_TRACE_ERROR("Trace level: %d\r\n", dcbm1_trace_lvl);
	}
	else if(data[0] == 'w' && length >= 2)
	{
		if(data[1] == 'i')
		{
			if(dcbm1_writesync())
				DCBM1_TRACE_ERROR("Send OK %d\r\n", tx_busy);
		}
		else if(data[1] == '0')
		{
			if(dcbm1_writereg(0, 0))
				DCBM1_TRACE_ERROR("Send OK %d\r\n", tx_busy);
		}
		else if(data[1] == 'w')
		{
			uint16_t res = dcbm1_send(&data[2], length - 2);
			DCBM1_TRACE_ERROR("Send %d bytes (%d)\r\n", res, dcbm1_extint_cnt)
		}
	}
	else if(data[0] == 'r' && length >= 2)
	{
		if(data[1] >= '0')
		{
			uint8_t bfr;
			if(dcbm1_readreg(data[1] - '0', &bfr))
				DCBM1_TRACE_ERROR("Read 0x%02x OK -> %02x\r\n", data[1] - '0', bfr);
		}
	}
	else if(data[0] == 'i' && length > 2)
	{
		int val = 0, max_wait = 1000;
		for(int i = 2; i < length; i++)
		{
			val *= 10;
			val += data[i] - '0';
		}

		while(tx_busy && max_wait-- > 0)
			vTaskDelay(2);

		if(data[1] == 'f')
			dcbm1_setfrequency(val);
		else if(data[1] == 'r')
			dcbm1_setrate(val);
	}
	else if(data[0] == 'l' && length >= 2)
	{
		tx_loop = data[1] - '0';
		DCBM1_TRACE_ERROR("Loop %d\r\n", tx_loop);
	}
	else if(data[0] == 'q' && length >= 1)
	{
		DCBM1_EN_TX(1);			// Indicate it can be send
		vTaskDelay(1);
		DCBM1_EN_TX(0);			// Indicate it can be send
		DCBM1_TRACE_ERROR("Send empty\r\n");
	}
}
#endif

void dcbm1_enable_bus_busy(bool enable)
{
	dcbm1_bus_busy_enable = enable;
}

uint16_t dcbm1_send(volatile uint8_t *bfr, uint16_t length)
{
	uint16_t amount = 0;
	if(length >= DCBM1_MAX_TX_BUFFER)
		return 0;

	if(tx_busy)
		DCBM1_TRACE_ERROR("Oeioei still busy\r\n");

	tx_busy = 1;
	
	xQueueReset(xDcbm1_TxSemaphore);
	DCBM1_UART_PUTBFR((uint8_t *)bfr, length);
		
	if(if_pin_func == 1)
	{
		if(xSemaphoreTake(xDcbm1_TxSemaphore, 20))		// Max wait 400 bytes, just a time
		{
			amount = length;
			if(dcbm1_delayed_tx)
			{
				uint8_t i = 0;
				for(i = 0; i < 4; i++)			// Wait for bus not busy
				{
					if(!DCBM1_CSEL1_BUS_BUSY)
					{
						DCBM1_EN_TX(1);					// Indicate it can be send
						break;
					}
					vTaskDelay(i + 1);
				}
				if(i >= 4)
				{
					DCBM1_EN_TX(1);						// Do it any way
					DCBM1_TRACE_ERROR("Delayed Fifo send: %d\r\n", i);
				}
				DCBM1_EN_TX(0);							// Set to idle again
			}
			tx_busy = 0;
			dcbm1_delayed_tx = false;
			return amount;
		}
	}
	dcbm1_delayed_tx = false;
	tx_busy = 0;
	return 0;
}

#define REG_WRITE_CMD	0xF5
#define REG_READ_CMD	0xFD

bool dcbm1_writesync()
{
	DCBM1_HDC(0);
	rx_commandmode = 1;

	uint8_t reg = REG_WRITE_CMD;
	uint8_t amount = dcbm1_send(&reg, 1);

	rx_commandmode = 0;
	vTaskDelay(1);
	DCBM1_HDC(1);

	return amount == 1;
}

bool dcbm1_writereg(uint8_t reg, uint8_t data)
{
	uint8_t bfr[3];
	bfr[0] = REG_WRITE_CMD;
	bfr[1] = reg;
	bfr[2] = data;

	DCBM1_HDC(0);
	_NOP();
	_NOP();
	rx_commandmode = 1;
	uint8_t amount = dcbm1_send(bfr, sizeof(bfr));

	vTaskDelay(2);
	rx_commandmode = 0;
	//DCBM1_HDC(1);

	return amount == 3;
}

bool dcbm1_writeonlyreg(uint8_t reg, uint8_t data)
{
	uint8_t bfr[3];
	bfr[0] = REG_WRITE_CMD;
	bfr[1] = reg;
	bfr[2] = data;

	DCBM1_HDC(0);
	_NOP();
	_NOP();
	rx_commandmode = 1;
	uint8_t amount = dcbm1_send(bfr, sizeof(bfr));

	rx_commandmode = 0;
	for(uint16_t s = 0; s < 250; s++)					// Wait until last byte is also sent
	{
		if(s == 125)
			taskYIELD();
		_NOP();
	}
	DCBM1_HDC(1);

	return amount == 3;
}

bool dcbm1_readreg(uint8_t reg, uint8_t *data)
{
	uint8_t bfr[2];
	bool res = false;

	bfr[0] = REG_READ_CMD;
	bfr[1] = reg;

	rx_rec = false;

	DCBM1_HDC(0);
	_NOP();
	_NOP();
	
	rx_commandmode = 1;
	xQueueReset(xDcbm1_RxSemaphore);
	dcbm1_send(bfr, 2);

	if(xSemaphoreTake(xDcbm1_RxSemaphore, 10))
	{
		//DCBM1_TRACE_ERROR("Rec OK [%d][%d]: %02x\r\n", rx_select_read, rx_index_read, rx_buffer[rx_select_read][0]);
		if(rx_rec)
		{
			*data = rx_data;
			res = true;
		}
	}
	rx_commandmode = 0;
	DCBM1_HDC(1);

	return res;
}

int8_t dcbm1_writeverifyreg(uint8_t reg, uint8_t data_write)
{
	if(dcbm1_writereg(reg, data_write))
	{
		uint8_t data_read;
		if(reg == 2)							// Setting Frequency select?
			vTaskDelay(2);						// Wait ff so it can stabilize, it won't react for 1ms

		if(dcbm1_readreg(reg, &data_read))		//
		{
			return data_write == data_read ? 0 : -3;
		}
		return -2;
	}
	return -1;
}

int8_t dcbm1_setfrequency(uint8_t frequency)
{
	int8_t res = dcbm1_writeverifyreg(2, frequency);
	DCBM1_TRACE_INFO("Changed frequency to: %d.%dMHz -> %s\r\n", (frequency/10) + 5, frequency % 10, res >= 0 ? "OK" : "Error");
	return res;
}

bool dcbm1_checkfrequency(uint8_t frequency)
{
	uint8_t data_read;
	if(dcbm1_readreg(2, &data_read))
		return data_read == frequency;
	
	return false;
}

int dcbm1_getfrequency_h(uint8_t frequency)
{
	return (frequency/10) + 5;
}

int dcbm1_getfrequency_l(uint8_t frequency)
{
	return frequency % 10;
}

const int csel_rates[4] = { 1400000, 1000000, 490000, 225000 };

int8_t dcbm1_setrate(uint8_t rate)
{
	int8_t res = dcbm1_writeverifyreg(0, 0x60 | (rate & 0x03));
	DCBM1_TRACE_INFO("Changed rate to: %d -> %s\r\n", csel_rates[(rate & 0x03)], res >= 0 ? "OK" : "Error");

	return res;
}

bool dcbm1_checkrate(uint8_t rate)
{
	uint8_t data_read;
	if(dcbm1_readreg(0, &data_read))
		return data_read == rate;
	
	return false;
}

int dcbm1_getrate_kHz(uint8_t rate)
{
	return csel_rates[(rate & 0x03)]/1000;
}

void dcbm1_checkerror()
{
#ifdef DCBM1_ERROR_ENABLE	
	if(dcbm1_extint_cnt)
	{
		State_SetWarning(STATE_WARNING_UART_OVW_WARN);
		dcbm1_extint_cnt = 0;
		dcbm1_extint_dly = 100;
	}
	else if(dcbm1_extint_dly == 0)
	{
		State_ClearWarning(STATE_WARNING_UART_OVW_WARN);
	}
	else
		dcbm1_extint_dly--;
#endif //DCBM1_ERROR_ENABLE		
}

int8_t dcbm1_settxlevelraw(uint8_t txlevel)
{
	int8_t res = dcbm1_writeverifyreg(1, txlevel);
	//DCBM1_TRACE_INFO("Changed tx level raw to: %d -> %s\r\n", txlevel, res >= 0 ? "OK" : "Error");

	return res;
}

void dcbm1_reset()
{
	DCBM1_RST(0);
	vTaskDelay(5);
	DCBM1_RST(1);
}

int8_t dcbm1_init(uint8_t frequency, uint8_t rate, uint8_t txlevel)
{
	//uint8_t csel = rate & 0x03;
	int8_t res = -1;

	if_pin_func = 0x00;				// 1 = EN_TX/BUS_BUSY, 0 = CSEL0/CSEL1
	tx_busy = 0;

	dcbm1_usart_init();
	
	for(int retry = 0; retry < 3; retry++)
	{
		DCBM1_HDC(1);
		DCBM1_EN_TX(0);						
	
		/*if(if_pin_func)
		{
			DCBM1_CSEL0_DIR(GPIO_DIRECTION_OUT);	// EN_TX
			DCBM1_CSEL1_DIR(GPIO_DIRECTION_IN);		// BUS_BUSY
			DCBM1_EN_TX(0);
		}
		else
		{
			DCBM1_CSEL0_DIR(GPIO_DIRECTION_OUT);	
			DCBM1_CSEL1_DIR(GPIO_DIRECTION_OUT);	
			DCBM1_CSEL0(csel & 0x01);		// Set lowest speed
			DCBM1_CSEL1((csel & 0x02) >> 1);
		}*/
	
		vTaskDelay(2);
		DCBM1_RST(0);
		vTaskDelay(5);
		DCBM1_RST(1);
		vTaskDelay(5);
		
		dcbm1_writesync();				// Sync on our bitrate
		
		vTaskDelay(2);
	
		if_pin_func = 0x01;				// 1 = EN_TX/BUS_BUSY, 0 = CSEL0/CSEL1 (check dcbm1_setrate if changed)
		if(if_pin_func)
		{
			if((res = dcbm1_writeverifyreg(3, 0x1C)) < 0)			// Default + EN_TX mode
			{
				DCBM1_TRACE_ERROR("Write verify fail on reg %d : %02x\r\n", 3, res);
			}
			DCBM1_CSEL0_DIR(GPIO_DIRECTION_OUT);	// EN_TX
			DCBM1_CSEL1_DIR(GPIO_DIRECTION_IN);		// BUS_BUSY
		}
		else
			res = 0;

		if(res >= 0)
			res = dcbm1_setrate(rate);
		if(res >= 0)			// Just a test to see if chip is ok
		{
			res = dcbm1_setfrequency(frequency);		// Set to default frequency
			if(res >= 0)
			{
				DCBM1_TRACE_ERROR("Init powerline OK @ bit-rate=%d (f: %d)\r\n", csel_rates[rate], frequency);
				res = dcbm1_settxlevelraw(txlevel);		// 0xF0 1Vpp, 0xF8 is 2Vpp
			}
			else
			{
				DCBM1_TRACE_ERROR("Init powerline Err2: %d\r\n", res);
			}
		}
		else
		{
			DCBM1_TRACE_ERROR("Init powerline Err: %d\r\n", res);
		}
		if(res < 0)
			State_SetError(STATE_ERROR_HW_ERROR);
		else
			State_ClearError(STATE_ERROR_HW_ERROR);
	
		DCBM1_HDC(1);
		
		if(res >= 0)
			break;
			
		vTaskDelay(2);
	}
	if(dcbm1_cb_func_rx != dcbm1_rx_cb && dcbm1_cb_func_rx != NULL)
		dcbm1_set_rx_callback(dcbm1_cb_func_rx);
	
	return res;
}

bool dcbm1_sleep()
{
	if(!tx_busy)
		return dcbm1_writeonlyreg(3, 0x9F);			// Default + EN_TX mode + Enter sleep mode + set deep sleep
	return false;
}

void dcbm1_wake()
{
	uint16_t s;
	DCBM1_HDC(0);
	for(s = 0; s < 375; s++)					// Should be at least 64us (measured 100us)
	{
		_NOP();
		if(s == 125 || s == 250)
			taskYIELD();
	}
	DCBM1_HDC(1);
}

/************************************************************************/
/* Start the DCBM1 task and init uart								        */
/************************************************************************/
void dcbm1_driver_init(void)
{
	xDcbm1_RxSemaphore = xSemaphoreCreateBinary();
	if(xDcbm1_RxSemaphore == NULL)
		DCBM1_TRACE_ERROR("Semaphore not created DCBM1 Rx");			// Todo: what to do

	xDcbm1_TxSemaphore = xSemaphoreCreateBinary();
	if(xDcbm1_TxSemaphore == NULL)
		DCBM1_TRACE_ERROR("Semaphore not created DCBM1 Tx");			// Todo: what to do
}


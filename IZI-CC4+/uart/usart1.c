/*
 * uart0.c
 *
 * Created: 19-8-2021 09:19:58
 *  Author: Milo
 */ 

#include "includes.h"
#include "usart1.h"
#include "version.h"
#if 0
#define USART_1_BUFFER_SIZE 1024

struct usart_async_descriptor USART_1;
struct io_descriptor *io;

static uint8_t USART_1_buffer[USART_1_BUFFER_SIZE];
static uint8_t USART_1_txbuffer[2][USART_1_BUFFER_SIZE];
static uint8_t USART_1_txbuffer_ptr = 0;
static volatile uint16_t USART_1_txbuffer_idx;


/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_1_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM1_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_1_PORT_init()
{
	gpio_set_pin_function(PA16, PINMUX_PA16C_SERCOM1_PAD0);
	gpio_set_pin_function(PA17, PINMUX_PA17C_SERCOM1_PAD1);
}

#if 0
static void tx_cb_USART_1(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}
#endif

static void rx_cb_USART_1(const struct usart_async_descriptor *const io_descr)
{
	//char pipo[32];
	/* Receive completed */
	//io_read(io_descr, pipo, sizeof(pipo));	
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART1_Init(void)
{
	USART_1_CLOCK_init();
	usart_async_init(&USART_1, SERCOM1, USART_1_buffer, USART_1_BUFFER_SIZE, (void *)NULL);
	USART_1_PORT_init();
	
	usart_async_register_callback(&USART_1, USART_ASYNC_TXC_CB, tx_cb_USART_1);
	usart_async_register_callback(&USART_1, USART_ASYNC_RXC_CB, rx_cb_USART_1);
	/*usart_async_register_callback(&USART_1, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_1, &io);
	usart_async_enable(&USART_1);
	
	USART_1_txbuffer_idx = 0;
	OS_TRACE_INFO("\r\nIZI-PowerCom: %d.%d.%d\r\n", firmare_info.version.v.major, firmare_info.version.v.minor, firmare_info.version.v.build);
}

void USART1_Putc(char c)
{
	if(USART_1_txbuffer_idx >= USART_1_BUFFER_SIZE)
	{
		USART1_Send();
	}
		
	USART_1_txbuffer[USART_1_txbuffer_ptr][USART_1_txbuffer_idx] = c;
	USART_1_txbuffer_idx++;
}

void USART1_Send()
{
	if(USART_1_txbuffer_idx > 0)
	{
		io_write(io, USART_1_txbuffer[USART_1_txbuffer_ptr], USART_1_txbuffer_idx);	
		if(USART_1_txbuffer_ptr == 0)
			USART_1_txbuffer_ptr = 1;
		else
			USART_1_txbuffer_ptr = 0;
		USART_1_txbuffer_idx = 0;
	}
}

bool USART1_Getc(char *c)
{
	uint8_t bfr[2];
	int32_t rec = io_read(io, bfr, 1);	
	if(rec > 0)
	{
		USART1_Putc(bfr[0]);
		*c = bfr[0];
		return true;
	}
	return false;
}
#endif

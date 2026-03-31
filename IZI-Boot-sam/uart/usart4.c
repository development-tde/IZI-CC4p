/*
 * uart0.c
 *
 * Created: 19-8-2021 09:19:58
 *  Author: Milo
 */ 

#include "includes.h"
#include "usart4.h"
#include "version.h"

#ifdef UART4_USED

#define USART_4_BUFFER_SIZE 1024

struct usart_async_descriptor USART_4;
struct io_descriptor *io;

static uint8_t USART_4_buffer[USART_4_BUFFER_SIZE];
static uint8_t USART_4_txbuffer[2][USART_4_BUFFER_SIZE];
static uint8_t USART_4_txbuffer_ptr = 0;
static volatile uint16_t USART_4_txbuffer_idx;


/*! The buffer size for USART */
/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_4_CLOCK_init()
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBDMASK_SERCOM4_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_4_PORT_init()
{
	gpio_set_pin_function(PB08, PINMUX_PB08D_SERCOM4_PAD0);

	gpio_set_pin_function(PB09, PINMUX_PB09D_SERCOM4_PAD1);
}

#if 0
static void tx_cb_USART_4(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}
#endif

static void rx_cb_USART_4(const struct usart_async_descriptor *const io_descr)
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
void USART4_Init(void)
{
	USART_4_CLOCK_init();
	usart_async_init(&USART_4, SERCOM4, USART_4_buffer, USART_4_BUFFER_SIZE, (void *)NULL);
	USART_4_PORT_init();
	
	//usart_async_register_callback(&USART_4, USART_ASYNC_TXC_CB, tx_cb_USART_4);
	usart_async_register_callback(&USART_4, USART_ASYNC_RXC_CB, rx_cb_USART_4);
	/*usart_async_register_callback(&USART_4, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_4, &io);
	usart_async_enable(&USART_4);
	
	USART_4_txbuffer_idx = 0;
	OS_TRACE_INFO("\r\nIZI-PowerCom bootloader: %d.%d.%d\r\n", version.v.major, version.v.minor, version.v.build);
}

void USART4_Putc(char c)
{
	if(USART_4_txbuffer_idx >= USART_4_BUFFER_SIZE)
	{
		USART4_Send();
	}
		
	USART_4_txbuffer[USART_4_txbuffer_ptr][USART_4_txbuffer_idx] = c;
	USART_4_txbuffer_idx++;
}

void USART4_Send()
{
	if(USART_4_txbuffer_idx > 0)
	{
		io_write(io, USART_4_txbuffer[USART_4_txbuffer_ptr], USART_4_txbuffer_idx);	
		if(USART_4_txbuffer_ptr == 0)
			USART_4_txbuffer_ptr = 1;
		else
			USART_4_txbuffer_ptr = 0;
		USART_4_txbuffer_idx = 0;
	}
}

bool USART4_Getc(char *c)
{
	uint8_t bfr[2];
	int32_t rec = io_read(io, bfr, 1);	
	if(rec > 0)
	{
		USART4_Putc(bfr[0]);
		*c = bfr[0];
		return true;
	}
	return false;
}

#endif


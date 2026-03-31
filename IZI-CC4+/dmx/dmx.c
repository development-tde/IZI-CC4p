/*
 * usart1.c
 *
 * Created: 8-3-2022 08:42:33
 *  Author: Milo
 */ 

#include "includes.h"
#include "dmx.h"
#include "appconfig.h"
#include "usart2.h"
#include "rdm.h"
#include "dmx_rdm.h"
#include "iziplus_module_def.h"
#include "izi_output.h"
#include "dmx_rdm_models.h"
#include "dmx_rdm_module.h"

// Defines
#define RS485_TX()		PORT->Group[EXT_RW_PORT].OUTSET.reg = 0x01 << EXT_RW_PIN;
#define RS485_RX()		PORT->Group[EXT_RW_PORT].OUTCLR.reg = 0x01 << EXT_RW_PIN;

#define TASK_DMXIN_STACK_SIZE		(2048 / sizeof(portSTACK_TYPE))
#define TASK_DMXIN_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

#define DMXIN_TIIMEOUT_100MS		30			// 3 second timeout

// Globals
static volatile uint8_t			dmxin_channels[DMXIN_MAX_CHANNELS];
static volatile uint16_t		dmxin_idx, dmxin_total, dmxin_fixture_chan_end, dmxin_fixture_chan_start, dmxin_fixture_chan_amount;
static volatile dmxin_state_t	dmxin_state;
static volatile uint16_t		dmxin_dump;
static volatile uint16_t		dmxin_valid_timer, dmxin_valid_test;
static volatile uint8_t			dmxin_valid;
static volatile bool			dmxRdmPacketReady;
static volatile uint8_t			dmxin_tx_buffer[DMXRDM_MAX_CHANNELS];
static volatile uint16_t		dmxin_tx_length;

static TaskHandle_t		xDmxIn_Task;
static SemaphoreHandle_t xDmxIn_Data_RxCmdSemaphore = NULL;

// Proto
void Dmx_TxReadyInt();
void Dmx_RecvInt(uint8_t c);
void Dmx_BreakInt();
static void DmxIn_Task(void *p);

/************************************************************************/
/* Start                                                                */
/************************************************************************/
void Dmx_Init()
{
	dmxin_state = DMXIN_STATE_BREAK;
	dmxin_idx = 0;
	dmxin_valid_timer = 0;
	dmxin_valid = false;
	dmxRdmPacketReady = false;
	
	Usart2_Init(Dmx_TxReadyInt, Dmx_RecvInt, Dmx_BreakInt);
	//Usart2_BreakEnable(Dmx_BreakInt);
	DmxRdm_Init();
	
	RS485_RX();
	
	if(xDmxIn_Data_RxCmdSemaphore == NULL)
		xDmxIn_Data_RxCmdSemaphore = xSemaphoreCreateBinary();
	
	
	if(xDmxIn_Task == NULL)
	{
		if (xTaskCreate(DmxIn_Task, "DmxIn", TASK_DMXIN_STACK_SIZE, NULL, TASK_DMXIN_TASK_PRIORITY, &xDmxIn_Task) != pdPASS) {
			while (1) {
				;
			}
		}
	}
}

static void DmxIn_Task(void *p)
{
	while(true)
	{
		if(xSemaphoreTake(xDmxIn_Data_RxCmdSemaphore, 1000))
		{
			if(dmxRdmPacketReady)										// RDM packet received?
			{
				dmxRdmPacketReady = false;
				DmxRdm_FrameHandler();									// Handle the frame
				DmxRdm_Module_Delayed();								// Check if actions have to executed delayed
			}
		}
		else
			DmxRdm_Module_Timer();
	}
}

/**
 * Force a break state (self timed), call dmxin_breakstop when break should end
 */
void Dmx_BreakStart()
{
	Usart2_BreakOn();
}

/**
 * Force break stop
 */
void Dmx_BreakStop()
{
	Usart2_BreakOff();
}

inline void Dmx_Direction(bool tx)
{
	if(tx)
	{	RS485_TX();	}
	else
	{	RS485_RX();	}
}

uint16_t Dmx_SendPrepare(volatile uint8_t *bfr, uint16_t length)
{
	if(length > DMXRDM_MAX_CHANNELS)
		return 0;
		
	memcpy((uint8_t *)dmxin_tx_buffer, (uint8_t *)bfr, length);
	dmxin_tx_length = length;
	
	return dmxin_tx_length;
}

uint16_t Dmx_Send()
{
	Usart2_PutBfr((uint8_t *)dmxin_tx_buffer, dmxin_tx_length);
	return dmxin_tx_length;
}

void Dmx_SendFlush()
{
	dmxin_tx_length = 0;
}

/************************************************************************/
/* Called ever 100ms (from IziPlus_Module_Timer100ms)                   */
/************************************************************************/
void Dmx_Timer100ms()
{
	if(dmxin_valid_timer > 0)
	{
		if(--dmxin_valid_timer == 0)
			dmxin_valid = 0;
	}	
}

/************************************************************************/
/* Check if DMX is still receiving (3 sec timeout after last frame)     */
/************************************************************************/
bool Dmx_Valid()
{
	return dmxin_valid_timer > 0;
}

/************************************************************************/
/* TX complete interrupt                                                */
/************************************************************************/
void Dmx_TxReadyInt()
{
	DmxRdm_SendReady();
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void Dmx_RecvInt(uint8_t c)
{
	if(dmxin_state == DMXIN_STATE_START)			// Break received?
	{
		if(c == E120_SC_RDM)
		{
			dmxin_state = DMXIN_STATE_REC_RDM;		// Possible RDM frame
			DmxRdm_Start();
			dmxin_valid_test++;
		}
		else if(c == 0)								// DMX frame
		{
			dmxin_state = DMXIN_STATE_REC;	
			
			dmxin_fixture_chan_amount = IziPlus_Module_GetChannelAmount();
			dmxin_fixture_chan_start = appConfig->channel;
			dmxin_fixture_chan_end = dmxin_fixture_chan_start + dmxin_fixture_chan_amount;		// One to high, but it is checked with a idx that is one too high
			
			if(dmxin_idx != dmxin_total || dmxin_idx == 0)		// Same amount twice?
			{
				dmxin_valid = 0;
				dmxin_total = dmxin_idx;
			}
			else
			{
				dmxin_valid = 1;
				dmxin_valid_timer = DMXIN_TIIMEOUT_100MS;		// Same amount twice? Report OK
			}
			dmxin_idx = 0;							// Reset receive counter
		}
		else
			dmxin_state = DMXIN_STATE_BREAK;		// Unknown
	}
	else if(dmxin_state == DMXIN_STATE_REC)
	{
		if(dmxin_idx < DMXIN_MAX_CHANNELS)
		{
			dmxin_channels[dmxin_idx++] = c;
			if(dmxin_valid > 0 && dmxin_idx == dmxin_fixture_chan_end)												// All bytes for this fixture in?
			{
				if((dmxin_fixture_chan_start + dmxin_fixture_chan_amount) == dmxin_fixture_chan_end)			// Check if really valid
				{
					Izi_OutputSetBuffer(IZIOUTPUT_SRC_DMX, 0, (uint8_t *)&dmxin_channels[dmxin_fixture_chan_start], dmxin_fixture_chan_amount);
					dmxin_valid = 2;	
					State_ComToggle();																		// Report set ok!
				}
			}
		}
		else
			dmxin_state = DMXIN_STATE_BREAK;
	}
	else if(dmxin_state == DMXIN_STATE_REC_RDM)
	{
		uint8_t rdm_state = DmxRdm_DataIn(c);
		if(rdm_state == RDM_RX_READY)
		{
			// Report frame received
			dmxRdmPacketReady = true;
			State_ComToggleRdm();
			xSemaphoreGiveFromISR(xDmxIn_Data_RxCmdSemaphore, NULL);
			portEND_SWITCHING_ISR(true);			// Make sure the semaphore is directly handled (iso the next tick)
			dmxin_state = DMXIN_STATE_BREAK;
		}
	}	
}

void Dmx_BreakInt()
{
	dmxin_state = DMXIN_STATE_START;				// Start receiving bytes
}

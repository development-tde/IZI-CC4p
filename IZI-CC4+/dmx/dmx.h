/*
 * usartx.h
 *
 * Created: 8-2-2025 08:42:52
 *  Author: Milo
 */ 


#ifndef _DMX_H_
#define _DMX_H_

#define DMXIN_MAX_CHANNELS			512
#define DMXRDM_MAX_CHANNELS			260		// Max is 257
#define DMXIN_RX_TIMEOUT			100

typedef enum {
	DMXIN_STATE_BREAK = 0,			// Waiting for break
	DMXIN_STATE_START = 1,			// Waiting for start char
	DMXIN_STATE_REC = 2,
	DMXIN_STATE_ERR = 3,
	DMXIN_STATE_REC_RDM = 0x10,
} dmxin_state_t;

// Proto
void Dmx_Init();
void Dmx_Timer100ms();
bool Dmx_Valid();
void Dmx_BreakStart();
void Dmx_BreakStop();
void Dmx_Direction(bool tx);
uint16_t Dmx_SendPrepare(volatile uint8_t *bfr, uint16_t length);
uint16_t Dmx_Send();
void Dmx_SendFlush();

#endif /* _DMX_H_ */
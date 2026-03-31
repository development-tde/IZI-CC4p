/*
 * timing.c
 *
 * Created: 15-2-2025 16:25:27
 *  Author: Milo
 */ 
#include "includes.h"
#include "timing.h"
#include "timer_def.h"

#ifndef TIMER_MODULE
#define TIMER_MODULE		TCC4
#define TIMER_OVF_HANDLER	TCCX_INT_OVF_HANDLER(4)
#define TIMER_MCMP0_HANDLER	TCCX_INT_MCMP0_HANDLER(4)
#define TIMER_MCMP1_HANDLER	TCCX_INT_MCMP1_HANDLER(4)

#define TIMER_IRQ_OVF		TCCX_IRQ_OVF_DEF(4)
#define TIMER_IRQ_MCMP0		TCCX_IRQ_MCMP0_DEF(4)
#define TIMER_IRQ_MCMP1		TCCX_IRQ_MCMP1_DEF(4)

#define TIMER_GLCK_FREQ		96000000
#define TIMER_GLCK_DIV		TCC_CTRLA_PRESCALER_DIV16
#define TIMER_FREQ			6000000  // 6 MHz (0.16666us per tick)
#define TIMER_PRIO			3
#define TIMER_APBXMASK		((Mclk *)MCLK)->APBDMASK.bit.TCC4_
#define TIMER_GCLK_ID		TCCX_GCLK_ID(4)
#endif
#define TIMER_MICRO_SEC_PRS	(TIMER_FREQ / 1000000)

static void (*callback1)(void) = NULL;
static void (*callback2)(void) = NULL;

void TIMER_MCMP0_HANDLER(void) {
	if (TIMER_MODULE->INTFLAG.bit.MC0 && callback1) {
		callback1();
		callback1 = NULL;
		TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC0;  // Clear interrupt flag
		TIMER_MODULE->INTENCLR.reg = TCC_INTENCLR_MC0;  // Disable interrupt
	}
	else
		TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC0;  // Clear interrupt flag
}

void TIMER_MCMP1_HANDLER(void) {
	if (TIMER_MODULE->INTFLAG.bit.MC1 && callback2) {
		callback2();
		callback2 = NULL;
		TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC1;  // Clear interrupt flag
		TIMER_MODULE->INTENCLR.reg = TCC_INTENCLR_MC1;  // Disable interrupt
	}
	else
		TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC1;  // Clear interrupt flag
}

void timing_init(void) {
	// Timer init (TCCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TIMER_APBXMASK = 1;
	MCLK_CRITICAL_SECTION_LEAVE();

	hri_gclk_write_PCHCTRL_reg(GCLK, TIMER_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 0) = 96MHz
	
	TIMER_MODULE->CTRLA.bit.SWRST = 1;
	while (TIMER_MODULE->SYNCBUSY.bit.SWRST);

	TIMER_MODULE->CTRLA.reg = TIMER_GLCK_DIV;			// Prescaler to get x MHz
	TIMER_MODULE->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ;		// Normal frequency mode
	TIMER_MODULE->PER.bit.PER = 0xFFFF;
	TIMER_MODULE->CTRLA.bit.ENABLE = 1;
	while (TIMER_MODULE->SYNCBUSY.bit.ENABLE);
	
	TIMER_MODULE->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	
	NVIC_DisableIRQ(TIMER_IRQ_MCMP0);
	NVIC_ClearPendingIRQ(TIMER_IRQ_MCMP0);
	NVIC_SetPriority(TIMER_IRQ_MCMP0, TIMER_PRIO);			// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TIMER_IRQ_MCMP0);
	
	NVIC_DisableIRQ(TIMER_IRQ_MCMP1);
	NVIC_ClearPendingIRQ(TIMER_IRQ_MCMP1);
	NVIC_SetPriority(TIMER_IRQ_MCMP1, TIMER_PRIO);			// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TIMER_IRQ_MCMP1);
}

uint16_t timing_get_count(void) {
	TIMER_MODULE->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
	
	while (TIMER_MODULE->SYNCBUSY.bit.CTRLB);

	return (TIMER_MODULE->COUNT.bit.COUNT) / TIMER_MICRO_SEC_PRS;
}

void timing_set_callback0(uint16_t us, void (*callback)(void)) {
	TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC0;  // Clear interrupt flag before enabling

	callback1 = callback;
	TIMER_MODULE->COUNT.bit.COUNT = 0;
	TIMER_MODULE->CC[0].bit.CC = (/*timing_get_count() +*/ (us * TIMER_MICRO_SEC_PRS)) & 0xFFFF;
	TIMER_MODULE->INTENSET.reg = TCC_INTENSET_MC0;
}

void timing_set_callback1(uint16_t us, void (*callback)(void)) {
	TIMER_MODULE->INTFLAG.reg = TCC_INTFLAG_MC1;  // Clear interrupt flag before enabling

	callback2 = callback;
	TIMER_MODULE->COUNT.bit.COUNT = 0;
	TIMER_MODULE->CC[1].bit.CC = (/*timing_get_count() + */(us * TIMER_MICRO_SEC_PRS)) & 0xFFFF;
	TIMER_MODULE->INTENSET.reg = TCC_INTENSET_MC1;
}
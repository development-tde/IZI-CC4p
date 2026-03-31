/*
 * timer_def.h
 *
 * Created: 15-3-2022 14:28:59
 *  Author: Milo
 */ 


#ifndef TIMER_DEF_H_
#define TIMER_DEF_H_

#define TC_SELECT			0				// Own define for selecting type of timer (TC or TCC)
#define TCC_SELECT			1

#define GCLK_GEN0_FREQ		96000000		// 96MHz based on DPLL0
#define GCLK_GEN1_FREQ		12000000		// 12MHz based on DPLL0
#define GCLK_GEN2_FREQ		1000000			// 1MHz based on XOSC (is OFF!!!! check gclk settings) 
#define GCLK_GEN3_FREQ		120000000		// 120MHz based on DPLL1

#define TC0_GCLK_ID                 9
#define TC1_GCLK_ID                 9
#define TC3_GCLK_ID                 26      // Index of Generic Clock
#define TCC0_GCLK_ID                25
#define TCC1_GCLK_ID                25		// Shared with TCC0
#define TCC2_GCLK_ID                29      // Index of Generic Clock
#define TCC3_GCLK_ID                29
#define AC_GCLK_ID					32
#define DAC_GCLK_ID					42


// General helper defines
#define TCX_IRQ_DEF(t)			TC##t##_IRQn
#define TCX_DEF(t)				TC##t
#define TCX_EVSYS_ID_GEN_OVF(t)	EVSYS_ID_GEN_TC##t##_OVF
#define TCX_EVSYS_ID_GEN_MCMP(t)	EVSYS_ID_GEN_TC##t##_MC_0
#define TCX_EVSYS_ID_GEN_MCMP1(t)	EVSYS_ID_GEN_TC##t##_MC_1
#define TCX_GCLK_ID(t)			TC##t##_GCLK_ID
#define TCX_INT_HANDLER(t)		TC##t##_Handler

#define TCCX_IRQ_OVF_DEF(t)		TCC##t##_0_IRQn
#define TCCX_IRQ_MCMP0_DEF(t)	TCC##t##_1_IRQn
#define TCCX_IRQ_MCMP1_DEF(t)	TCC##t##_2_IRQn
#define TCCX_IRQ_MCMP2_DEF(t)	TCC##t##_3_IRQn
#define TCCX_IRQ_MCMP3_DEF(t)	TCC##t##_4_IRQn
#define TCCX_DEF(t)				TCC##t
#define TCCX_EVSYS_ID_GEN_OVF(t) EVSYS_ID_GEN_TCC##t##_OVF
#define TCCX_EVSYS_ID_GEN_MCMP(t)	EVSYS_ID_GEN_TCC##t##_MC_0
#define TCCX_EVSYS_ID_GEN_MCMP1(t)	EVSYS_ID_GEN_TCC##t##_MC_1
#define TCCX_EVSYS_ID_GEN_MCMP2(t)	EVSYS_ID_GEN_TCC##t##_MC_2
#define TCCX_EVSYS_ID_GEN_MCMP3(t)	EVSYS_ID_GEN_TCC##t##_MC_3 
#define TCCX_GCLK_ID(t)			TCC##t##_GCLK_ID
#define TCCX_INT_OVF_HANDLER(t)	TCC##t##_0_Handler
#define TCCX_INT_MCMP0_HANDLER(t)	TCC##t##_1_Handler
#define TCCX_INT_MCMP1_HANDLER(t)	TCC##t##_2_Handler
#define TCCX_INT_MCMP2_HANDLER(t)	TCC##t##_3_Handler
#define TCCX_INT_MCMP3_HANDLER(t)	TCC##t##_4_Handler


#endif /* TIMER_DEF_H_ */
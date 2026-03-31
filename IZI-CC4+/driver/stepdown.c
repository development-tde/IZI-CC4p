/*
 * stepdown.c
 *
 * Created: 3-3-2022 13:39:31
 *  Author: Milo
 */ 

#include "includes.h"
#include "stepdown.h"
#include "timer_def.h"
#include "izi_output.h"
#include "curve.h"
#include "analog.h"
#include "appconfig.h"
#include "iziplus_module_def.h"
#include "limits.h"
#include "mcp4728.h"
#include "at30tse.h"
#include "i2cx.h"
#include "izi_input.h"

#define TASK_STEPDOWN_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_STEPDOWN_TASK_PRIORITY (tskIDLE_PRIORITY + 6)

#define STEPDOWN_FREQ			100000	//0				// 100kHz
#define STEPDOWN_TICK_US		(1000000 / STEPDOWN_FREQ)	// 10us
#define STEPDOWN_OFF_TIME		0.000001				// 1us	(If changed, check STEPDOWN_SAFETY_TIME, STEPDOWN_OFF_TIME should be +/- 85% of STEPDOWN_SAFETY_TIME
#define STEPDOWN_SAFETY_TIME	0.0000066				// 8us	(when AC does not trigger, safety off)

#define PWM_START_LEVEL_MAX		5						// Start of frequency increase
#define PWM_LVL_OC_DETECT		32						// PWM level where open circuit is checked by vled (should be smaller than VLED_SC_ERROR_DBC)
#define PWM_START_LEVEL_FREQ	(72)					// Level for start of frequency increase start
#define PWM_LVL_SHORT_DETECT	250						// Filter Level value from where short circuit will be checked		
#define PWM_START_FREQ			1850UL					// 1850Hz start frequency
#define PWM_END_FREQ			28000UL					// 22000Hz end/highest frequency
#define PWM_FREQ				PWM_START_FREQ	

#define DAC_MIN					512						// Minimum DAC value for OK measurement
#define DAC_MIN_VIRTUAL			512						// Minimum DAC value that we want to match other IZI+ products, but can't be met due to bad DAC
#define DAC_MAX					4095					// Maximum DAC value for OK measurement

#define POWER_MAX				40000					// Max in power in mW

#define VIN_WARNING_DBC			4						// Debounce for vin warning (per 10 ms)
#define VLED_OC_WARNING_DBC		4						// Debounce for open circuit warning (per 10 ms)
#define VLED_SC_ERROR_DBC		40						// Debounce for short circuit error (per 10 ms) (should be larger than PWM_LVL_OC_DETECT)

#define SAFETY_OFF_OPEN_MS		1000					// 1 sec off when open output detected
#define SAFETY_OFF_SHORT_MS		5000					// 5 sec off when short on output detected

#ifndef _NVM_SW_CALIB_AREA_BASE
#define _NVM_SW_CALIB_AREA_BASE 0x00800080
#endif

#define STEPDOWN_TIMEROS_TICKS		10					// Amount of ticks for 10 msecond timer

#define MIN_VIN_MV				30000					// Stop operation if vin lower than 37V (38V on again when 1V hysteresis)
#define WARN_VIN_MV				30000					// Warn user for low supply (on at 40V when hysteresis is 1V)
#define MIN_VIN_HYST_MV			1000					// Hysteresis when on again
#define MAX_VIN_MV				52000					// High vin detected


// Change numbers for other timer TCCx used for channel 1 (x is 0 to 2). WARNING ONLY TCC0 and TCC1 are 24-bit (others are 16-bit). This means 16-bit timers (running @ 120MHz, only support 120MHz/65535= 1831Hz not lower!!!
#define TCCX_IRQ_OVF_EN_CH1			TCCX_IRQ_OVF_DEF(3)
#define TCCX_TIMER_EN_CH1			TCCX_DEF(3)
#define TCCX_GCLK_ID_EN_CH1			TCCX_GCLK_ID(3)
#define TCCX_EVSYS_ID_OVF_EN_CH1	TCCX_EVSYS_ID_GEN_OVF(3)
#define TCCX_EVSYS_ID_USER_EV0_CH1	EVSYS_ID_USER_TCC3_EV_0
#define TCCX_EVSYS_ID_USER_EV1_CH1	EVSYS_ID_USER_TCC3_EV_1
#define TCCX_APBXMASK_EN_CH1		((Mclk *)MCLK)->APBCMASK.bit.TCC3_
#define TCCX_INTO_HANDLER_EN_CH1	TCCX_INT_OVF_HANDLER(3)
#define TCCX_INTC_HANDLER_EN_CH1	TCCX_INT_MCMP0_HANDLER(3)		// CHECK EN WO when changed!!!!
#define TCCX_INTC_INTENSET_CH1		TCC_INTENSET_OVF// TCC_INTENSET_MC2				// CHECK EN WO when changed!!!!
#define TCCX_DRVCTRL_INV_CH1		TCC_DRVCTRL_INVEN1				// CHECK EN WO when changed!!!!
#define TCCX_CLOCK_FREQ_EN_CH1		(GCLK_GEN3_FREQ)				// If changed, check clock settings in code

// Change numbers for other timer TCCx used for channel 2 (x is 0 to 2). WARNING ONLY TCC0 and TCC1 are 24-bit (others are 16-bit). This means 16-bit timers (running @ 120MHz, only support 120MHz/65535= 1831Hz not lower!!!
#define TCCX_IRQ_OVF_EN_CH2			TCCX_IRQ_OVF_DEF(1)
#define TCCX_TIMER_EN_CH2			TCCX_DEF(1)
#define TCCX_GCLK_ID_EN_CH2			TCCX_GCLK_ID(1)
#define TCCX_EVSYS_ID_OVF_EN_CH2	TCCX_EVSYS_ID_GEN_OVF(1)
#define TCCX_EVSYS_ID_USER_EV0_CH2	EVSYS_ID_USER_TCC1_EV_0
#define TCCX_EVSYS_ID_USER_EV1_CH2	EVSYS_ID_USER_TCC1_EV_1
#define TCCX_APBXMASK_EN_CH2		((Mclk *)MCLK)->APBBMASK.bit.TCC1_
#define TCCX_INTO_HANDLER_EN_CH2	TCCX_INT_OVF_HANDLER(1)
#define TCCX_INTC_HANDLER_EN_CH2	TCCX_INT_MCMP2_HANDLER(1)		// CHECK EN WO when changed!!!!
#define TCCX_INTC_INTENSET_CH2		TCC_INTENSET_OVF				// CHECK EN WO when changed!!!!
#define TCCX_DRVCTRL_INV_CH2		TCC_DRVCTRL_INVEN3				// CHECK EN WO when changed!!!!
#define TCCX_CLOCK_FREQ_EN_CH2		(GCLK_GEN3_FREQ)				// If changed, check clock settings in code

// Change numbers for other timer TCCx used for channel 3 (x is 0 to 2). WARNING ONLY TCC0 and TCC1 are 24-bit (others are 16-bit). This means 16-bit timers (running @ 120MHz, only support 120MHz/65535= 1831Hz not lower!!!
#define TCCX_IRQ_OVF_EN_CH3			TCCX_IRQ_OVF_DEF(2)
#define TCCX_TIMER_EN_CH3			TCCX_DEF(2)
#define TCCX_GCLK_ID_EN_CH3			TCCX_GCLK_ID(2)
#define TCCX_EVSYS_ID_OVF_EN_CH3	TCCX_EVSYS_ID_GEN_OVF(2)
#define TCCX_EVSYS_ID_USER_EV0_CH3	EVSYS_ID_USER_TCC2_EV_0
#define TCCX_EVSYS_ID_USER_EV1_CH3	EVSYS_ID_USER_TCC2_EV_1
#define TCCX_APBXMASK_EN_CH3		((Mclk *)MCLK)->APBCMASK.bit.TCC2_
#define TCCX_INTO_HANDLER_EN_CH3	TCCX_INT_OVF_HANDLER(2)
#define TCCX_INTC_HANDLER_EN_CH3	TCCX_INT_MCMP2_HANDLER(2)		// CHECK EN WO when changed!!!!
#define TCCX_INTC_INTENSET_CH3		TCC_INTENSET_OVF				// CHECK EN WO when changed!!!!
#define TCCX_DRVCTRL_INV_CH3		TCC_DRVCTRL_INVEN2				// CHECK EN WO when changed!!!!
#define TCCX_CLOCK_FREQ_EN_CH3		(GCLK_GEN3_FREQ)				// If changed, check clock settings in code

// Change numbers for other timer TCCx used for channel 4 (x is 0 to 2). WARNING ONLY TCC0 and TCC1 are 24-bit (others are 16-bit). This means 16-bit timers (running @ 120MHz, only support 120MHz/65535= 1831Hz not lower!!!
#define TCCX_IRQ_OVF_EN_CH4			TCCX_IRQ_OVF_DEF(0)
#define TCCX_TIMER_EN_CH4			TCCX_DEF(0)
#define TCCX_GCLK_ID_EN_CH4			TCCX_GCLK_ID(0)
#define TCCX_EVSYS_ID_OVF_EN_CH4	TCCX_EVSYS_ID_GEN_OVF(0)
#define TCCX_EVSYS_ID_USER_EV0_CH4	EVSYS_ID_USER_TCC3_EV_0
#define TCCX_EVSYS_ID_USER_EV1_CH4	EVSYS_ID_USER_TCC3_EV_1
#define TCCX_APBXMASK_EN_CH4		((Mclk *)MCLK)->APBBMASK.bit.TCC0_
#define TCCX_INTO_HANDLER_EN_CH4	TCCX_INT_OVF_HANDLER(0)
#define TCCX_INTC_HANDLER_EN_CH4	TCCX_INT_MCMP0_HANDLER(0)		// CHECK EN WO when changed!!!!
#define TCCX_INTC_INTENSET_CH4		TCC_INTENSET_OVF				// CHECK EN WO when changed!!!!
#define TCCX_DRVCTRL_INV_CH4		TCC_DRVCTRL_INVEN0				// CHECK EN WO when changed!!!!
#define TCCX_CLOCK_FREQ_EN_CH4		(GCLK_GEN3_FREQ)				// If changed, check clock settings in code

// Change numbers for other timer TCx used for channel 1 (x is 0 to 3)
#define TCX_IRQ_CH1				TCX_IRQ_DEF(3)
#define TCX_TIMER_CH1			TCX_DEF(3)
#define TCX_GCLK_ID_CH1			TCX_GCLK_ID(3)
#define TCX_EVSYS_ID_OVF_CH1	TCX_EVSYS_ID_GEN_OVF(3)
#define TCX_EVSYS_ID_MCMP_CH1	TCX_EVSYS_ID_GEN_MCMP1(3)
#define TCX_EVSYS_ID_USER_CH1	EVSYS_ID_USER_TC3_EVU
#define TCX_APBXMASK_CH1		((Mclk *)MCLK)->APBBMASK.bit.TC3_
#define TCX_INT_HANDLER_CH1		TCX_INT_HANDLER(3)
#define TCX_CLOCK_FREQ_CH1		(GCLK_GEN3_FREQ)			// If changed, check clock settings in code

// Change numbers for other timer TCx used for channel 2 (x is 0 to 3)
#define TCX_IRQ_CH2				TCX_IRQ_DEF(1)
#define TCX_TIMER_CH2			TCX_DEF(1)
#define TCX_GCLK_ID_CH2			TCX_GCLK_ID(1)
#define TCX_EVSYS_ID_OVF_CH2	TCX_EVSYS_ID_GEN_OVF(1)
#define TCX_EVSYS_ID_MCMP_CH2	TCX_EVSYS_ID_GEN_MCMP1(1)
#define TCX_EVSYS_ID_USER_CH2	EVSYS_ID_USER_TC1_EVU
#define TCX_APBXMASK_CH2		((Mclk *)MCLK)->APBAMASK.bit.TC1_
#define TCX_INT_HANDLER_CH2		TCX_INT_HANDLER(1)
#define TCX_CLOCK_FREQ_CH2		TCX_CLOCK_FREQ_CH1			// If changed, check clock settings in code

// Change numbers for other timer TCx used for channel 2 (x is 0 to 3)
#define TCX_IRQ_CH3				TCX_IRQ_DEF(2)
#define TCX_TIMER_CH3			TCX_DEF(2)
#define TCX_GCLK_ID_CH3			TCX_GCLK_ID(2)
#define TCX_EVSYS_ID_OVF_CH3	TCX_EVSYS_ID_GEN_OVF(2)
#define TCX_EVSYS_ID_MCMP_CH3	TCX_EVSYS_ID_GEN_MCMP1(2)
#define TCX_EVSYS_ID_USER_CH3	EVSYS_ID_USER_TC2_EVU
#define TCX_APBXMASK_CH3		((Mclk *)MCLK)->APBBMASK.bit.TC2_
#define TCX_INT_HANDLER_CH3		TCX_INT_HANDLER(2)
#define TCX_CLOCK_FREQ_CH3		TCX_CLOCK_FREQ_CH1			// If changed, check clock settings in code

// Change numbers for other timer TCx used for channel 2 (x is 0 to 3)
#define TCX_IRQ_CH4				TCX_IRQ_DEF(0)
#define TCX_TIMER_CH4			TCX_DEF(0)
#define TCX_GCLK_ID_CH4			TCX_GCLK_ID(0)
#define TCX_EVSYS_ID_OVF_CH4	TCX_EVSYS_ID_GEN_OVF(0)
#define TCX_EVSYS_ID_MCMP_CH4	TCX_EVSYS_ID_GEN_MCMP1(0)
#define TCX_EVSYS_ID_USER_CH4	EVSYS_ID_USER_TC0_EVU
#define TCX_APBXMASK_CH4		((Mclk *)MCLK)->APBAMASK.bit.TC0_
#define TCX_INT_HANDLER_CH4		TCX_INT_HANDLER(0)
#define TCX_CLOCK_FREQ_CH4		TCX_CLOCK_FREQ_CH1			// If changed, check clock settings in code

// LUT usage
#define CCL_LUT_CH1				 CCL->LUTCTRL[3]
#define CCL_LUT_CH2				 CCL->LUTCTRL[1]
#define CCL_LUT_CH3				 CCL->LUTCTRL[2]
#define CCL_LUT_CH4				 CCL->LUTCTRL[0]

// Event
#define EVSYS_ID_GEN_CH1		EVSYS_ID_GEN_CCL_LUTOUT_3
#define EVSYS_ID_GEN_CH2		EVSYS_ID_GEN_CCL_LUTOUT_1
#define EVSYS_ID_GEN_CH3		EVSYS_ID_GEN_CCL_LUTOUT_2
#define EVSYS_ID_GEN_CH4		EVSYS_ID_GEN_CCL_LUTOUT_0

volatile uint32_t counter = 0, ac_ok = 0;

static uint16_t			time_after_pup = 0;		// in 10ms

uint16_t stepdown_current_chx = 0;

static TaskHandle_t		xStepDown_Task;
static TickType_t		last_log_ticks;

volatile stepdown_control_t stepdown_ctrl_chx[MAX_DIM_CHANNELS] = 
{
	{
		.channel = 1,
		.vled_func = Adc_GetVled1,
		.vled_raw_func = Adc_GetVled1Raw,
		.vled_last_func = Adc_GetVled1LastRaw,
		.set_current = Stepdown_SetCurrentCh1,
		//.temp_limit = IziPlus_Module_TempLimit_Chan1,
		//.issafety_off = IziPlus_Module_IsSafetyOff_Ch1,
		.channel_enable = StepDown_ChannelEnable_Ch1,
		.is_16b = true,
		.tcc_freq = TCCX_CLOCK_FREQ_EN_CH1,
		.vfled = 0
	},
	{
		.channel = 2,
		.vled_func = Adc_GetVled2,
		.vled_raw_func = Adc_GetVled2Raw,
		.vled_last_func = Adc_GetVled2LastRaw,
		.set_current = Stepdown_SetCurrentCh2,
		//.temp_limit = IziPlus_Module_TempLimit_Chan2,
		//.issafety_off = IziPlus_Module_IsSafetyOff_Ch2,
		.channel_enable = StepDown_ChannelEnable_Ch2,
		.is_16b = false,
		.tcc_freq = TCCX_CLOCK_FREQ_EN_CH2,
		.vfled = 0
	},
	{
		.channel = 3,
		.vled_func = Adc_GetVled3,
		.vled_raw_func = Adc_GetVled3Raw,
		.vled_last_func = Adc_GetVled3LastRaw,
		.set_current = Stepdown_SetCurrentCh3,
		//.temp_limit = IziPlus_Module_TempLimit_Chan3,
		//.issafety_off = IziPlus_Module_IsSafetyOff_Ch3,
		.channel_enable = StepDown_ChannelEnable_Ch3,
		.is_16b = true,
		.tcc_freq = TCCX_CLOCK_FREQ_EN_CH3,
		.vfled = 0
	},
	{
		.channel = 4,
		.vled_func = Adc_GetVled4,
		.vled_raw_func = Adc_GetVled4Raw,
		.vled_last_func = Adc_GetVled4LastRaw,
		.set_current = Stepdown_SetCurrentCh4,
		//.temp_limit = IziPlus_Module_TempLimit_Chan3,
		//.issafety_off = IziPlus_Module_IsSafetyOff_Ch4,
		.channel_enable = StepDown_ChannelEnable_Ch4,
		.is_16b = false,
		.tcc_freq = TCCX_CLOCK_FREQ_EN_CH4,
		.vfled = 0
	}
};

static uint16_t stepdown_closed = 0;
static uint8_t stepdown_force_update = 0;
static uint8_t stepdown_debug_level = 0;
static uint8_t stepdown_debug_channel = 0;
static uint8_t	stepdown_i2c_warning_time;

static volatile uint8_t stepdown_blank_short = 0;

static volatile uint32_t stepdown_power;

static bool				stepdown_task_fast, stepdown_task_fast_toggle;
//static uint16_t stepdown_test = 0;

// Proto
void Stepdown_SetCurrent();
void StepDown_StepTimer();
static void StepDown_Task(void *p);
bool StepDown_AnyChannelActive();
extern boot_t boot_data;

/************************************************************************/
/* Init Timer for channel 1. Timer is used for stepdown off time  xStepDown_Task      */
/************************************************************************/
void StepDown_InitTimer1()
{
	// Timer init (TCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCX_APBXMASK_CH1 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();

	hri_gclk_write_PCHCTRL_reg(GCLK, TCX_GCLK_ID_CH1, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = 120MHz

	TCX_TIMER_CH1->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while ((TCX_TIMER_CH1->COUNT16.SYNCBUSY.reg) & TC_SYNCBUSY_SWRST) {
	};

	TCX_TIMER_CH1->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val);				// 120MHz / 1 = 120MHz
	//TCX_TIMER_CH1->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_MC0;									// Overflow will only occur when AC does not trigger (safety)
	TCX_TIMER_CH1->COUNT16.WAVE.reg = 0x03;//TC_WAVE_WAVEGEN_MFRQ;						
	TCX_TIMER_CH1->COUNT16.CCBUF[0].reg = TCX_CLOCK_FREQ_CH1 * STEPDOWN_SAFETY_TIME; 
	//TCX_TIMER_CH1->COUNT16.CCBUF[1].reg = TCX_CLOCK_FREQ_CH1 * STEPDOWN_OFF_TIME;
	TCX_TIMER_CH1->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO | TC_EVCTRL_MCEO1 | TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_START;			// Output event on overflow and compare match (MUST BE SET BEFORE ENABLED!!)
	TCX_TIMER_CH1->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;	
	TCX_TIMER_CH1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	// Use 16-bit

	/*NVIC_DisableIRQ(TCX_IRQ_CH1);
	NVIC_ClearPendingIRQ(TCX_IRQ_CH1);
	NVIC_SetPriority(TCX_IRQ_CH1, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCX_IRQ_CH1);*/
}

/************************************************************************/
/* Init Timer for channel 2. Timer is used for stepdown off time        */
/************************************************************************/
void StepDown_InitTimer2()
{
	// Timer init (TCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCX_APBXMASK_CH2 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();

	hri_gclk_write_PCHCTRL_reg(GCLK, TCX_GCLK_ID_CH2, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = 120MHz

	TCX_TIMER_CH2->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while ((TCX_TIMER_CH2->COUNT16.SYNCBUSY.reg) & TC_SYNCBUSY_SWRST) {
	};

	TCX_TIMER_CH2->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val);				// 120MHz / 1 = 120MHz
	//TCX_TIMER_CH2->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_MC0;									// Overflow will only occur when AC does not trigger (safety)
	TCX_TIMER_CH2->COUNT16.WAVE.reg = 0x03;//TCC_WAVE_WAVEGEN_NPWM;						
	TCX_TIMER_CH2->COUNT16.CCBUF[0].reg = TCX_CLOCK_FREQ_CH2 * STEPDOWN_SAFETY_TIME; 
	//TCX_TIMER_CH2->COUNT16.CCBUF[1].reg = TCX_CLOCK_FREQ_CH2 * STEPDOWN_OFF_TIME;
	TCX_TIMER_CH2->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO | TC_EVCTRL_MCEO1 | TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_START;			// Output event on overflow and compare match (MUST BE SET BEFORE ENABLED!!)
	TCX_TIMER_CH2->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;	
	TCX_TIMER_CH2->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	// Use 16-bit

	/*NVIC_DisableIRQ(TCX_IRQ_CH2);
	NVIC_ClearPendingIRQ(TCX_IRQ_CH2);
	NVIC_SetPriority(TCX_IRQ_CH2, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCX_IRQ_CH2);*/
}

/************************************************************************/
/* Init Timer for channel 3. Timer is used for stepdown off time        */
/************************************************************************/
void StepDown_InitTimer3()
{
	// Timer init (TCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCX_APBXMASK_CH3 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();

	hri_gclk_write_PCHCTRL_reg(GCLK, TCX_GCLK_ID_CH3, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = 120MHz

	TCX_TIMER_CH3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while ((TCX_TIMER_CH3->COUNT16.SYNCBUSY.reg) & TC_SYNCBUSY_SWRST) {
	};

	TCX_TIMER_CH3->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val);				// 120MHz / 1 = 120MHz
	//TCX_TIMER_CH3->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_MC0;									// Overflow will only occur when AC does not trigger (safety)
	TCX_TIMER_CH3->COUNT16.WAVE.reg = 0x03;//TCC_WAVE_WAVEGEN_NPWM;						
	TCX_TIMER_CH3->COUNT16.CCBUF[0].reg = TCX_CLOCK_FREQ_CH3 * STEPDOWN_SAFETY_TIME; 
	//TCX_TIMER_CH2->COUNT16.CCBUF[1].reg = TCX_CLOCK_FREQ_CH3 * STEPDOWN_OFF_TIME;
	TCX_TIMER_CH3->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO | TC_EVCTRL_MCEO1 | TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_START;			// Output event on overflow and compare match (MUST BE SET BEFORE ENABLED!!)
	TCX_TIMER_CH3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;	
	TCX_TIMER_CH3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	// Use 16-bit

	/*NVIC_DisableIRQ(TCX_IRQ_CH2);
	NVIC_ClearPendingIRQ(TCX_IRQ_CH2);
	NVIC_SetPriority(TCX_IRQ_CH2, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCX_IRQ_CH2);*/
}

/************************************************************************/
/* Init Timer for channel 4. Timer is used for stepdown off time        */
/************************************************************************/
void StepDown_InitTimer4()
{
	// Timer init (TCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCX_APBXMASK_CH4 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();

	hri_gclk_write_PCHCTRL_reg(GCLK, TCX_GCLK_ID_CH4, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = 120MHz

	TCX_TIMER_CH4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while ((TCX_TIMER_CH4->COUNT16.SYNCBUSY.reg) & TC_SYNCBUSY_SWRST) {
	};

	TCX_TIMER_CH4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val);				// 120MHz / 1 = 120MHz
	//TCX_TIMER_CH4->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_MC0;									// Overflow will only occur when AC does not trigger (safety)
	TCX_TIMER_CH4->COUNT16.WAVE.reg = 0x03;//TCC_WAVE_WAVEGEN_NPWM;						
	TCX_TIMER_CH4->COUNT16.CCBUF[0].reg = TCX_CLOCK_FREQ_CH4 * STEPDOWN_SAFETY_TIME; 
	//TCX_TIMER_CH2->COUNT16.CCBUF[1].reg = TCX_CLOCK_FREQ_CH4 * STEPDOWN_OFF_TIME;
	TCX_TIMER_CH4->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO | TC_EVCTRL_MCEO1 | TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_START;			// Output event on overflow and compare match (MUST BE SET BEFORE ENABLED!!)
	TCX_TIMER_CH4->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;	
	TCX_TIMER_CH4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	// Use 16-bit

	/*NVIC_DisableIRQ(TCX_IRQ_CH2);
	NVIC_ClearPendingIRQ(TCX_IRQ_CH2);
	NVIC_SetPriority(TCX_IRQ_CH2, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCX_IRQ_CH2);*/
}


/************************************************************************/
/* Init Timer for PWM channel 1. Timer is used for level duty cycle on DIM1 */
/************************************************************************/
void StepDown_InitPwmChannel1()
{
	// Timer init (TCCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCCX_APBXMASK_EN_CH1 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, TCCX_GCLK_ID_EN_CH1, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = 120MHz
	
	TCCX_TIMER_EN_CH1->CTRLA.reg = TCC_CTRLA_SWRST;
	while ((TCCX_TIMER_EN_CH1->SYNCBUSY.reg) & TCC_SYNCBUSY_SWRST) {
	};
	
	TCCX_TIMER_EN_CH1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
	TCCX_TIMER_EN_CH1->INTENSET.reg = TCCX_INTC_INTENSET_CH1;
	TCCX_TIMER_EN_CH1->PER.bit.PER = 0xFFFFFF;//(GCLK_GEN3_FREQ / PWM_FREQ) / 1;
	TCCX_TIMER_EN_CH1->CC[DIM1_WO].reg = 0xFFFFE0;				// Super short
	TCCX_TIMER_EN_CH1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;								// Normal PWM
	TCCX_TIMER_EN_CH1->EVCTRL.reg = /*TCC_EVCTRL_EVACT0_START | TCC_EVCTRL_EVACT1_STOP | TCC_EVCTRL_TCEI0 | TCC_EVCTRL_TCEI1 |*/ TCC_EVCTRL_OVFEO;	// Start/Stop via event system
/*#ifdef CC2_22021601
	TCCX_TIMER_EN_CH1->DRVCTRL.reg |= TCCX_DRVCTRL_INV_CH1;
#endif*/
	//TCCX_TIMER_EN_CH1->DRVCTRL.reg |= TCC_DRVCTRL_INVEN0;
	//TCCX_TIMER_EN_CH1->CTRLA.reg |= TCC_CTRLA_ENABLE;
	
	NVIC_DisableIRQ(TCCX_IRQ_OVF_EN_CH1);
	NVIC_ClearPendingIRQ(TCCX_IRQ_OVF_EN_CH1);
	NVIC_SetPriority(TCCX_IRQ_OVF_EN_CH1, 0);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCCX_IRQ_OVF_EN_CH1);
}

/************************************************************************/
/* Init Timer for PWM channel 2. Timer is used for level duty cycle on DIM2 */
/************************************************************************/
void StepDown_InitPwmChannel2()
{
	// Timer init (TCCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCCX_APBXMASK_EN_CH2 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, TCCX_GCLK_ID_EN_CH2, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 0) = 120MHz
	
	TCCX_TIMER_EN_CH2->CTRLA.reg = TCC_CTRLA_SWRST;
	while ((TCCX_TIMER_EN_CH2->SYNCBUSY.reg) & TCC_SYNCBUSY_SWRST) {
	};
	
	TCCX_TIMER_EN_CH2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
	TCCX_TIMER_EN_CH2->INTENSET.reg = TCCX_INTC_INTENSET_CH2;
	TCCX_TIMER_EN_CH2->PER.bit.PER = 0xFFFFFF;//(GCLK_GEN3_FREQ / PWM_FREQ) / 1;
	TCCX_TIMER_EN_CH2->CC[DIM2_WO].reg = 0xFFFFE0;				// Set later on
	TCCX_TIMER_EN_CH2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;								// Normal PWM
	TCCX_TIMER_EN_CH2->EVCTRL.reg = /*TCC_EVCTRL_EVACT0_START | TCC_EVCTRL_EVACT1_STOP | TCC_EVCTRL_TCEI0 | TCC_EVCTRL_TCEI1 |*/ TCC_EVCTRL_OVFEO;	// Start/Stop via event system
/*#ifdef CC2_22021601
	TCCX_TIMER_EN_CH2->DRVCTRL.reg |= TCCX_DRVCTRL_INV_CH2;
#endif*/
	//TCCX_TIMER_EN_CH2->CTRLA.reg |= TCC_CTRLA_ENABLE;
	
	NVIC_DisableIRQ(TCCX_IRQ_OVF_EN_CH2);
	NVIC_ClearPendingIRQ(TCCX_IRQ_OVF_EN_CH2);
	NVIC_SetPriority(TCCX_IRQ_OVF_EN_CH2, 0);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCCX_IRQ_OVF_EN_CH2);
}

/************************************************************************/
/* Init Timer for PWM channel 3. Timer is used for level duty cycle on DIM3 */
/************************************************************************/
void StepDown_InitPwmChannel3()
{
	// Timer init (TCCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCCX_APBXMASK_EN_CH3 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, TCCX_GCLK_ID_EN_CH3, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 0) = 120MHz
	
	TCCX_TIMER_EN_CH3->CTRLA.reg = TCC_CTRLA_SWRST;
	while ((TCCX_TIMER_EN_CH3->SYNCBUSY.reg) & TCC_SYNCBUSY_SWRST) {
	};
	
	TCCX_TIMER_EN_CH3->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
	TCCX_TIMER_EN_CH3->INTENSET.reg = TCCX_INTC_INTENSET_CH3;
	TCCX_TIMER_EN_CH3->PER.bit.PER = 0xFFFF;//(GCLK_GEN3_FREQ / PWM_FREQ) / 1;
	TCCX_TIMER_EN_CH3->CC[DIM3_WO].reg = 0xFFE0;				// Set later on
	TCCX_TIMER_EN_CH3->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;								// Normal PWM
	TCCX_TIMER_EN_CH3->EVCTRL.reg = /*TCC_EVCTRL_EVACT0_START | TCC_EVCTRL_EVACT1_STOP | TCC_EVCTRL_TCEI0 | TCC_EVCTRL_TCEI1 | */TCC_EVCTRL_OVFEO;	// Start/Stop via event system
/*#ifdef CC2_22021601
	TCCX_TIMER_EN_CH3->DRVCTRL.reg |= TCCX_DRVCTRL_INV_CH3;
#endif*/
	//TCCX_TIMER_EN_CH3->CTRLA.reg |= TCC_CTRLA_ENABLE;
	
	NVIC_DisableIRQ(TCCX_IRQ_OVF_EN_CH3);
	NVIC_ClearPendingIRQ(TCCX_IRQ_OVF_EN_CH3);
	NVIC_SetPriority(TCCX_IRQ_OVF_EN_CH3, 0);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCCX_IRQ_OVF_EN_CH3);
}

/************************************************************************/
/* Init Timer for PWM channel 4. Timer is used for level duty cycle on DIM4 */
/************************************************************************/
void StepDown_InitPwmChannel4()
{
	// Timer init (TCCx)
	MCLK_CRITICAL_SECTION_ENTER();
	TCCX_APBXMASK_EN_CH4 = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, TCCX_GCLK_ID_EN_CH4, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 0) = 120MHz
	
	TCCX_TIMER_EN_CH4->CTRLA.reg = TCC_CTRLA_SWRST;
	while ((TCCX_TIMER_EN_CH4->SYNCBUSY.reg) & TCC_SYNCBUSY_SWRST) {
	};
	
	TCCX_TIMER_EN_CH4->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
	TCCX_TIMER_EN_CH4->INTENSET.reg = TCCX_INTC_INTENSET_CH4;
	TCCX_TIMER_EN_CH4->PER.bit.PER = 0xFFFF;//(GCLK_GEN3_FREQ / PWM_FREQ) / 1;
	TCCX_TIMER_EN_CH4->CC[DIM4_WO].reg = 0xFFE0;				// Set later on
	TCCX_TIMER_EN_CH4->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;								// Normal PWM
	TCCX_TIMER_EN_CH4->EVCTRL.reg = /*TCC_EVCTRL_EVACT0_START | TCC_EVCTRL_EVACT1_STOP | TCC_EVCTRL_TCEI0 | TCC_EVCTRL_TCEI1 |*/ TCC_EVCTRL_OVFEO;	// Start/Stop via event system
/*#ifdef CC2_22021601
	TCCX_TIMER_EN_CH4->DRVCTRL.reg |= TCCX_DRVCTRL_INV_CH4;
#endif*/
	//TCCX_TIMER_EN_CH4->CTRLA.reg |= TCC_CTRLA_ENABLE;
	
	NVIC_DisableIRQ(TCCX_IRQ_OVF_EN_CH4);
	NVIC_ClearPendingIRQ(TCCX_IRQ_OVF_EN_CH4);
	NVIC_SetPriority(TCCX_IRQ_OVF_EN_CH4, 0);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(TCCX_IRQ_OVF_EN_CH4);
}

/************************************************************************/
/* Init channel 1 AC/Timer/Event system                                 */
/************************************************************************/
void StepDown_InitChannel1()
{
	StepDown_InitTimer1();
	StepDown_InitPwmChannel1();
	
	// Pin
	uint32_t oldval = PORT->Group[SW1_PORT].EVCTRL.reg;
	PORT->Group[SW1_PORT].EVCTRL.reg = oldval | PORT_EVCTRL_PORTEI0 | PORT_EVCTRL_EVACT0_OUT | PORT_EVCTRL_PID0(SW1_PIN);
												 /*PORT_EVCTRL_PORTEI1 | PORT_EVCTRL_EVACT1_CLR | PORT_EVCTRL_PID1(SW1_PIN) |
												 PORT_EVCTRL_PORTEI2 | PORT_EVCTRL_EVACT2_TGL | PORT_EVCTRL_PID2(SHUNT1_PIN);*/
	
	EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(AC_EXT_IN1_EVENT_ID_GEN);		// External int x
	EVSYS->Channel[1].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CH1);				// LUTOUT_0
	
	EVSYS->USER[EVSYS_ID_USER_PORT_EV_0].reg = EVSYS_USER_CHANNEL(1 + 1);			// Set port high (set in PORT->Group[SW1_PORT].EVCTRL)
	EVSYS->USER[TCX_EVSYS_ID_USER_CH1].reg = EVSYS_USER_CHANNEL(0 + 1);				// Retrigger/start timer
	
	EVSYS->SWEVT.reg = EVSYS_SWEVT_CHANNEL0;			// Start TC, needed for RETRIGGER mode
	
	// Comparator
	//AC->COMPCTRL[0].reg = (SNS1_COMPCTRL_MUXPOS) | (DAC_EXT_AC_MUXNEG2) | (AC_COMPCTRL_INTSEL_RISING) | AC_COMPCTRL_FLEN_MAJ3 | AC_COMPCTRL_OUT_ASYNC;
	//AC->COMPCTRL[0].reg |= AC_COMPCTRL_ENABLE | AC_COMPCTRL_SPEED(3);
	//AC->EVCTRL.reg |= AC_EVCTRL_COMPEO0;
	/*AC->INTENSET.reg = AC_INTENSET_COMP0;
	
	NVIC_DisableIRQ(AC_IRQn);
	NVIC_ClearPendingIRQ(AC_IRQn);
	NVIC_SetPriority(AC_IRQn, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(AC_IRQn);
	*/
	// CCL system (init basics)
	MCLK_CRITICAL_SECTION_ENTER();
	((Mclk *)MCLK)->APBCMASK.bit.CCL_ = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, CCL_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 0) = CONF_CPU_FREQUENCY = 120MHz
	
	CCL_LUT_CH1.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	
	gpio_set_pin_direction(SW1, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(SW1, GPIO_PULL_OFF);
	gpio_set_pin_function(SW1, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[SW1_PORT].OUTCLR.reg = 0x01 << SW1_PIN;
	
	gpio_set_pin_direction(DIM1, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(DIM1, GPIO_PULL_OFF);
	gpio_set_pin_function(DIM1, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[DIM1_PORT].OUTSET.reg = 0x01 << DIM1_PIN;
}

/************************************************************************/
/* Init channel 2 AC/Timer/Event system                                 */
/************************************************************************/
void StepDown_InitChannel2()
{
	StepDown_InitTimer2();
	StepDown_InitPwmChannel2();
	
	// Pin
	uint32_t oldval = PORT->Group[SW2_PORT].EVCTRL.reg;
	PORT->Group[SW2_PORT].EVCTRL.reg = oldval | PORT_EVCTRL_PORTEI2 | PORT_EVCTRL_EVACT2_OUT | PORT_EVCTRL_PID2(SW2_PIN) /*|
												 PORT_EVCTRL_PORTEI3 | PORT_EVCTRL_EVACT3_CLR | PORT_EVCTRL_PID3(SW2_PIN) |
												 PORT_EVCTRL_PORTEI2 | PORT_EVCTRL_EVACT2_CLR | PORT_EVCTRL_PID2(SW1_PIN)*/;
	
	EVSYS->Channel[2].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(AC_EXT_IN2_EVENT_ID_GEN);		// External int x
	EVSYS->Channel[3].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CH2);				// LUTOUT_1
	
	EVSYS->USER[EVSYS_ID_USER_PORT_EV_2].reg = EVSYS_USER_CHANNEL(3 + 1);			// Set port high (set in PORT->Group[SW2_PORT].EVCTRL)
	EVSYS->USER[TCX_EVSYS_ID_USER_CH2].reg = EVSYS_USER_CHANNEL(2 + 1);				// Retrigger/start timer
	
	EVSYS->SWEVT.reg = EVSYS_SWEVT_CHANNEL2;			// Start TC, needed for RETRIGGER mode

	// Comparator
	//AC->COMPCTRL[1].reg = (SNS2_COMPCTRL_MUXPOS) | (DAC_EXT_AC_MUXNEG) | (AC_COMPCTRL_INTSEL_RISING) | AC_COMPCTRL_FLEN_MAJ3 | AC_COMPCTRL_OUT_ASYNC;
	//AC->COMPCTRL[1].reg |= AC_COMPCTRL_ENABLE | AC_COMPCTRL_SPEED(3);
	//AC->EVCTRL.reg |= AC_EVCTRL_COMPEO1;
	/*AC->INTENSET.reg = AC_INTENSET_COMP1;
	
	NVIC_DisableIRQ(AC_IRQn);
	NVIC_ClearPendingIRQ(AC_IRQn);
	NVIC_SetPriority(AC_IRQn, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(AC_IRQn);
	*/
	
	CCL_LUT_CH2.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	
	gpio_set_pin_direction(SW2, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(SW2, GPIO_PULL_OFF);
	gpio_set_pin_function(SW2, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[SW2_PORT].OUTCLR.reg = 0x01 << SW2_PIN;
	
	gpio_set_pin_direction(DIM2, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(DIM2, GPIO_PULL_OFF);
	gpio_set_pin_function(DIM2, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[DIM2_PORT].OUTSET.reg = 0x01 << DIM2_PIN;
}

/************************************************************************/
/* Init channel 3 AC/Timer/Event system                                 */
/************************************************************************/
void StepDown_InitChannel3()
{
	StepDown_InitTimer3();
	StepDown_InitPwmChannel3();
	
	// Pin
	uint32_t oldval = PORT->Group[SW3_PORT].EVCTRL.reg;
	PORT->Group[SW3_PORT].EVCTRL.reg = oldval | PORT_EVCTRL_PORTEI1 | PORT_EVCTRL_EVACT1_OUT | PORT_EVCTRL_PID1(SW3_PIN) ;
	
	EVSYS->Channel[4].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(AC_EXT_IN3_EVENT_ID_GEN);		// External int x
	EVSYS->Channel[5].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CH3);				// LUTOUT_2
	
	EVSYS->USER[EVSYS_ID_USER_PORT_EV_1].reg = EVSYS_USER_CHANNEL(5 + 1);			// Set port high (set in PORT->Group[SW3_PORT].EVCTRL)
	EVSYS->USER[TCX_EVSYS_ID_USER_CH3].reg = EVSYS_USER_CHANNEL(4 + 1);				// Retrigger/start timer
	
	EVSYS->SWEVT.reg = EVSYS_SWEVT_CHANNEL4;			// Start TC, needed for RETRIGGER mode

	// Comparator
	//AC->COMPCTRL[1].reg = (SNS2_COMPCTRL_MUXPOS) | (DAC_EXT_AC_MUXNEG) | (AC_COMPCTRL_INTSEL_RISING) | AC_COMPCTRL_FLEN_MAJ3 | AC_COMPCTRL_OUT_ASYNC;
	//AC->COMPCTRL[1].reg |= AC_COMPCTRL_ENABLE | AC_COMPCTRL_SPEED(3);
	//AC->EVCTRL.reg |= AC_EVCTRL_COMPEO1;
	/*AC->INTENSET.reg = AC_INTENSET_COMP1;
	
	NVIC_DisableIRQ(AC_IRQn);
	NVIC_ClearPendingIRQ(AC_IRQn);
	NVIC_SetPriority(AC_IRQn, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(AC_IRQn);
	*/
	
	CCL_LUT_CH3.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	
	gpio_set_pin_direction(SW3, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(SW3, GPIO_PULL_OFF);
	gpio_set_pin_function(SW3, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[SW3_PORT].OUTCLR.reg = 0x01 << SW3_PIN;
	
	gpio_set_pin_direction(DIM3, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(DIM3, GPIO_PULL_OFF);
	gpio_set_pin_function(DIM3, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[DIM3_PORT].OUTSET.reg = 0x01 << DIM3_PIN;
}

/************************************************************************/
/* Init channel 4 AC/Timer/Event system                                 */
/************************************************************************/
void StepDown_InitChannel4()
{
	StepDown_InitTimer4();
	StepDown_InitPwmChannel4();
	
	// Pin
	uint32_t oldval = PORT->Group[SW4_PORT].EVCTRL.reg;
	PORT->Group[SW4_PORT].EVCTRL.reg = oldval | PORT_EVCTRL_PORTEI3 | PORT_EVCTRL_EVACT3_OUT | PORT_EVCTRL_PID3(SW4_PIN) ;
	
	EVSYS->Channel[6].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(AC_EXT_IN4_EVENT_ID_GEN);		// External int x
	EVSYS->Channel[7].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CH4);				// LUTOUT_3
	
	EVSYS->USER[EVSYS_ID_USER_PORT_EV_3].reg = EVSYS_USER_CHANNEL(7 + 1);			// Set port high (set in PORT->Group[SW3_PORT].EVCTRL)
	EVSYS->USER[TCX_EVSYS_ID_USER_CH4].reg = EVSYS_USER_CHANNEL(6 + 1);				// Retrigger/start timer
	
	EVSYS->SWEVT.reg = EVSYS_SWEVT_CHANNEL6;			// Start TC, needed for RETRIGGER mode

	// Comparator
	//AC->COMPCTRL[1].reg = (SNS2_COMPCTRL_MUXPOS) | (DAC_EXT_AC_MUXNEG) | (AC_COMPCTRL_INTSEL_RISING) | AC_COMPCTRL_FLEN_MAJ3 | AC_COMPCTRL_OUT_ASYNC;
	//AC->COMPCTRL[1].reg |= AC_COMPCTRL_ENABLE | AC_COMPCTRL_SPEED(3);
	//AC->EVCTRL.reg |= AC_EVCTRL_COMPEO1;
	/*AC->INTENSET.reg = AC_INTENSET_COMP1;
	
	NVIC_DisableIRQ(AC_IRQn);
	NVIC_ClearPendingIRQ(AC_IRQn);
	NVIC_SetPriority(AC_IRQn, 1);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(AC_IRQn);
	*/
	
	CCL_LUT_CH4.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	
	gpio_set_pin_direction(SW4, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(SW4, GPIO_PULL_OFF);
	gpio_set_pin_function(SW4, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[SW4_PORT].OUTCLR.reg = 0x01 << SW4_PIN;
	
	gpio_set_pin_direction(DIM4, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(DIM4, GPIO_PULL_OFF);
	gpio_set_pin_function(DIM4, GPIO_PIN_FUNCTION_OFF);
	PORT->Group[DIM4_PORT].OUTSET.reg = 0x01 << DIM4_PIN;
	
	/*gpio_set_pin_direction(DIM4_IN, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(DIM4_IN, GPIO_PULL_OFF);
	gpio_set_pin_function(DIM4_IN, DIM4_IN_FUNC);*/
}

void StepDown_Init()
{
	// Event system (init basics)
	MCLK_CRITICAL_SECTION_ENTER();
	((Mclk *)MCLK)->APBBMASK.bit.EVSYS_ = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	EVSYS->CTRLA.reg = EVSYS_CTRLA_SWRST;
	
	// Comparator (init basics, enable is later)
	//MCLK_CRITICAL_SECTION_ENTER();
	//((Mclk *)MCLK)->APBCMASK.bit.AC_ = 1;
	//MCLK_CRITICAL_SECTION_LEAVE();
	
	//hri_gclk_write_PCHCTRL_reg(GCLK, AC_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Main clock (Generic clock generator 3) = CONF_CPU_FREQUENCY = 120MHz
	
	/*AC->CTRLA.bit.SWRST = 1;
	while ((AC->SYNCBUSY.reg) & AC_SYNCBUSY_SWRST) {
	};
	AC->CALIB.reg = ((uint8_t *)_NVM_SW_CALIB_AREA_BASE)[0] & 0x03;		// Search for 'NVM Software Calibration Area Mapping' in datasheet
		*/
	CCL->CTRL.reg = CCL_CTRL_SWRST;
	
	StepDown_InitChannel1();
	StepDown_InitChannel2();
	StepDown_InitChannel3();
	StepDown_InitChannel4();

	// DAC
	Mcp4728_Init();
	
	// Enable CCL
	CCL->CTRL.reg = CCL_CTRL_ENABLE;
	
	Stepdown_SetCurrent();				// Set initial current
	
	//StepDown_Test();
	if(xStepDown_Task == NULL)
	{
		if (xTaskCreate(StepDown_Task, "StepDown", TASK_STEPDOWN_STACK_SIZE, NULL, TASK_STEPDOWN_TASK_PRIORITY, &xStepDown_Task) != pdPASS) {
			while (1) {
				;
			}
		}
	}
}

uint32_t Filter_LowPass(uint32_t old, uint32_t new, uint8_t shift)
{
	uint32_t filter;
	if(new != old)					
	{
		if(old < new)
		{
			filter = (uint32_t)(old + ((new - old) >> shift));
			if(old == filter)
				filter++;
		}
		else
		{
			filter = (uint32_t)(old - ((old - new) >> shift));
			if(old == filter)
				filter--;
		}
		old = filter;
	}
	else
	{
		filter = new;
	}
	return filter;
}

bool StepDown_IsClosed()
{
	return stepdown_closed > 0;	
}

/************************************************************************/
/* Close stepdown (= 0%)for time ms                                                                     */
/************************************************************************/
void StepDown_Close(uint16_t time)
{
	stepdown_closed = time / STEPDOWN_TIMEROS_TICKS;
	
	StepDown_ChannelEnable_Ch1(false, 0);
	StepDown_ChannelEnable_Ch2(false, 1);
	StepDown_ChannelEnable_Ch3(false, 2);
	StepDown_ChannelEnable_Ch4(false, 3);
} 


/************************************************************************/
/* Overflow interrupt of PWM timer to create random period			    */
/************************************************************************/
void TCCX_INTO_HANDLER_EN_CH1()
{
	volatile stepdown_control_t *ctrl = &stepdown_ctrl_chx[0];
	uint8_t cnt = ctrl->cnt, sel = ctrl->timing_sel > 0 ? 1 : 0;
		
	TCCX_TIMER_EN_CH1->PERBUF.reg = ctrl->per[sel][cnt];
	TCCX_TIMER_EN_CH1->CCBUF[0].reg = ctrl->cc0[sel][cnt];
	TCCX_TIMER_EN_CH1->CCBUF[DIM1_WO].reg = ctrl->ccx[sel][cnt];
	if(++ctrl->cnt >= MAX_TABLE)					// Goto the next
		ctrl->cnt = 0;				// Start over with the same random table (sync other channels as well)
	
	if(cnt == ctrl->switch_sel)						// Match to switch buffers?
	{
		ctrl->timing_sel = sel > 0 ? 0 : 1;			// Switch buffers
		ctrl->switch_sel = -1;						// Wait for next switch
	}
	
	TCCX_TIMER_EN_CH1->INTFLAG.reg = TCC_INTFLAG_OVF;
}

/************************************************************************/
/* Overflow interrupt of PWM timer to create random period			    */
/************************************************************************/
void TCCX_INTO_HANDLER_EN_CH2()
{
	volatile stepdown_control_t *ctrl = &stepdown_ctrl_chx[1];
	uint8_t cnt = ctrl->cnt, sel = ctrl->timing_sel > 0 ? 1 : 0;
		
	TCCX_TIMER_EN_CH2->PERBUF.reg = ctrl->per[sel][cnt];
	TCCX_TIMER_EN_CH2->CCBUF[0].reg = ctrl->cc0[sel][cnt];
	TCCX_TIMER_EN_CH2->CCBUF[DIM2_WO].reg = ctrl->ccx[sel][cnt];
	if(++ctrl->cnt >= MAX_TABLE)					// Goto the next
		ctrl->cnt = 0;				// Start over with the same random table (sync other channels as well)
	
	if(cnt == ctrl->switch_sel)						// Match to switch buffers?
	{
		ctrl->timing_sel = sel > 0 ? 0 : 1;			// Switch buffers
		ctrl->switch_sel = -1;						// Wait for next switch
	}
	TCCX_TIMER_EN_CH2->INTFLAG.reg = TCC_INTFLAG_OVF;
}

/************************************************************************/
/* Overflow interrupt of PWM timer to create random period			    */
/************************************************************************/
void TCCX_INTO_HANDLER_EN_CH3()
{
	volatile stepdown_control_t *ctrl = &stepdown_ctrl_chx[2];
	uint8_t cnt = ctrl->cnt, sel = ctrl->timing_sel > 0 ? 1 : 0;
	
	TCCX_TIMER_EN_CH3->PERBUF.reg = ctrl->per[sel][cnt];
	TCCX_TIMER_EN_CH3->CCBUF[0].reg = ctrl->cc0[sel][cnt];
	TCCX_TIMER_EN_CH3->CCBUF[DIM3_WO].reg = ctrl->ccx[sel][cnt];
	if(++ctrl->cnt >= MAX_TABLE)					// Goto the next
		ctrl->cnt = 0;				// Start over with the same random table (sync other channels as well)
	
	if(cnt == ctrl->switch_sel)						// Match to switch buffers?
	{
		ctrl->timing_sel = sel > 0 ? 0 : 1;			// Switch buffers
		ctrl->switch_sel = -1;						// Wait for next switch
	}
	
	TCCX_TIMER_EN_CH3->INTFLAG.reg = TCC_INTFLAG_OVF;
}
/************************************************************************/
/* Overflow interrupt of PWM timer to create random period			    */
/************************************************************************/
void TCCX_INTO_HANDLER_EN_CH4()
{
	volatile stepdown_control_t *ctrl = &stepdown_ctrl_chx[3];
	uint8_t cnt = ctrl->cnt, sel = ctrl->timing_sel > 0 ? 1 : 0;
	
	TCCX_TIMER_EN_CH4->PERBUF.reg = ctrl->per[sel][cnt];
	TCCX_TIMER_EN_CH4->CCBUF[0].reg = ctrl->cc0[sel][cnt];
	TCCX_TIMER_EN_CH4->CCBUF[DIM4_WO].reg = ctrl->ccx[sel][cnt];
	if(++ctrl->cnt >= MAX_TABLE)					// Goto the next
		ctrl->cnt = 0;				// Start over with the same random table (sync other channels as well)

	if(cnt == ctrl->switch_sel)						// Match to switch buffers?
	{
		ctrl->timing_sel = sel > 0 ? 0 : 1;			// Switch buffers
		ctrl->switch_sel = -1;						// Wait for next switch
	}
	
	TCCX_TIMER_EN_CH4->INTFLAG.reg = TCC_INTFLAG_OVF;
}

#define MAX_CFG_MA	19

uint16_t StepDown_GetCurrent()
{
	uint8_t idx = CONFIG_CURRENT_MA_RAW(appConfig->config);
	if(idx >= MAX_CFG_MA)
		idx = 0;
	
	return CONFIG_CURRENT_MA(appConfig->config);
}

void StepDown_Module_Update()
{
	uint16_t mA = StepDown_GetCurrent();
	if(mA != stepdown_current_chx)
	{
		Stepdown_SetCurrent(mA);
	}
}

bool dir = false;
uint8_t last_level = 0;

void StepDown_Debug(uint8_t* data, uint16_t length)
{
	if(length >= 2)
	{
		if(data[0] == 'C')
		{
			appconfig_t *cfgram = AppConfig_Get();
			cfgram->mode = 2;			// Set fixed to 2 channel mode
			AppConfig_Set();
		}
		else if(data[0] == 'T')
		{
			stepdown_debug_level = (data[1] - '0');
			stepdown_debug_channel = (data[2] - '0');
			OS_TRACE_ERROR("Debug Lvl: %d, Chan: %d\r\n", stepdown_debug_level, stepdown_debug_channel);
		}
	}
}


static uint8_t vin_error_prs = 0, vin_high_warn_prs = 0, power_high_warn_prs = 0, vin_warn_prs;

void StepDown_CheckLimits()
{
	if(time_after_pup < 25)			// Wait until everything is stable, skip the first 250ms after power-up
		return;
	
	uint32_t vin = Analog_GetSupplyVoltage();// Adc_GetVin();
	if(vin < MIN_VIN_MV)
	{
		if(++vin_error_prs >= (2*VIN_WARNING_DBC))
		{
			State_SetError(STATE_ERROR_VIN_LOW);
			vin_error_prs = VIN_WARNING_DBC;			// Set again after 4 ticks
			StepDown_Close(2000);
		}
	}
	else if(vin >= (MIN_VIN_MV + MIN_VIN_HYST_MV))
	{
		if(vin_error_prs > 0)
		{
			if(--vin_error_prs == 0)
				State_ClearError(STATE_ERROR_VIN_LOW);
		}
	}
	else if(vin_error_prs > 0)
		StepDown_Close(1000);					// Still to too low? Extend off time
	
	if(vin_error_prs == 0)
	{
		if(vin < WARN_VIN_MV)
		{
			if(++vin_warn_prs >= (2*VIN_WARNING_DBC))
			{
				State_SetWarning(STATE_WARNING_VIN_LOW);
				vin_warn_prs = VIN_WARNING_DBC;			// Set again after 4 ticks
			}
		}
		else if(vin >= (WARN_VIN_MV + (MIN_VIN_HYST_MV/2)))
		{
			if(vin_warn_prs > 0)
			{
				if(--vin_warn_prs == 0)
					State_ClearWarning(STATE_WARNING_VIN_LOW);
			}
		}
	}		
	
	if(vin > (MAX_VIN_MV + MIN_VIN_HYST_MV))
	{
		if(++vin_high_warn_prs >= VIN_WARNING_DBC)
		{
			State_SetWarning(STATE_WARNING_SUPPLY_HIGH);
			vin_high_warn_prs = VIN_WARNING_DBC;			// Clip here
			
		}
	}
	else if(vin < MAX_VIN_MV)
	{
		if(vin_high_warn_prs > 0)
		{
			if(--vin_high_warn_prs == 0)
				State_ClearWarning(STATE_WARNING_SUPPLY_HIGH);
		}
	}
	
	if(IziPlus_Module_PowerCheck())
	{
		if(++power_high_warn_prs >= 10)							// More than 100ms too high?
			State_SetWarning(STATE_WARNING_POWER1_HIGH);
	}
	else
	{
		if(power_high_warn_prs > 0)
		{
			if(--power_high_warn_prs == 0)
				State_ClearWarning(STATE_WARNING_POWER1_HIGH);
		}
	}
}

#define TOFF_MIN	78			// 650ns
#define CONST_A		185UL		// 25% ripple
#define CONST_B		14UL
#define CONST_C		407UL
#define CONST_D		12UL
#define CONST_E		182500UL
#define CONST_F		67UL
#define CONST_G		3341UL
#define CONST_H		0UL
#define CONST_I		3478UL
#define CONST_J		407UL
#define CONST_K		210UL
#define CONST_L		67UL
#define CONST_M		3341UL
#define CONST_N		0UL
#define CONST_O		4994UL
#define CONST_P		0UL
#define CONST_Q		255UL
#define CONST_U		5UL

#define MIN_VLED_MV				3000					// Minimal Vled voltage (also used for short detection)

static uint8_t vled_prs = 0;

/* Check if dac value is within limits */
uint32_t Stedown_DacLimit(uint32_t dac_data)
{
	if(dac_data < DAC_MIN)
		dac_data = DAC_MIN;
	else if(dac_data > DAC_MAX)
		dac_data = DAC_MAX;
	
	return dac_data;
}

void Stepdown_SetCurrentCalc(uint16_t mA, volatile  stepdown_control_t *ctrl, uint8_t chan_idx)
{
	uint16_t mA_org = mA;
	uint16_t vled_ad = ctrl->vled_raw_func();
	uint16_t vin_ad = Analog_GetSupplyVoltageRaw();
	
	if(vled_ad < ADC_MV_RAW(MIN_VLED_MV))
		vled_ad = ADC_MV_RAW(MIN_VLED_MV);										// If too low, use 5V so calculation will not fail
	
	// Top detector
	if((vled_ad > ctrl->vled_ad) || (vled_prs == 0))							// Fast up
	{
		uint16_t vled_old = ctrl->vled_ad;
		if(vled_old < vled_ad)
		{
			if(ctrl->filter_fast_speed <= 1)
			{
				ctrl->vled_ad = (uint16_t)(vled_old + ((vled_ad - vled_old) >> 2));
				if(vled_old == ctrl->vled_ad)
					ctrl->vled_ad++;
			}
			else
				ctrl->vled_ad = vled_ad;
		}
		else
		{
			ctrl->vled_ad = (uint16_t)(vled_old - ((vled_old - vled_ad) >> 4));
			if(vled_old == ctrl->vled_max)
				ctrl->vled_ad--;
		}
	}
	if(ctrl->vled_ad > ctrl->vled_max)
		ctrl->vled_max = ctrl->vled_ad;
	if((ctrl->vled_ad > ctrl->vled_max_session) && (ctrl->level >= 0xC000))		// Only store if more than 75% level
	{
		if(++ctrl->vled_max_session_prs >= 100)					// At least 800ms above 75% and more than previous max?
			ctrl->vled_max_session = ctrl->vled_ad;
	}
	else
		ctrl->vled_max_session_prs = 0;
	
	ctrl->vin_ad = vin_ad;
	if(ctrl->vin_ad < ctrl->vled_ad)
		ctrl->vled_ad = ctrl->vin_ad;
	
	mA = ((mA_org * (ctrl->level_curve16)) / 65536UL);
	
	uint32_t toff_min = TOFF_MIN;//CONST_A - ((vled_ad * CONST_A) / vin_ad);
		
	uint32_t toff = (((uint64_t)mA * (uint64_t)CONST_C) / (uint64_t)vled_ad);
	if(toff < toff_min)
		toff = toff_min;
		
	ctrl->toff_min = toff_min;
	ctrl->toff = toff;
	uint32_t toff_set = toff > CONST_D ? (toff - CONST_D) : 0;
	
	mA = mA_org;
	stepdown_current_chx = mA_org;
	
	/*uint32_t dac_max = 0;
	uint32_t par1 = (((mA * ctrl->level_curve) >> 16) * CONST_E) + ((toff_set + CONST_D) * vled_ad * CONST_F);
	uint32_t par2 = ((vin_ad - vled_ad) * CONST_G) + CONST_H;
	if(par1 > par2)
		dac_max = (par1 - par2) >> 16;
	
	uint8_t max_level_shift = 3;//5 - div;
	
	uint16_t dac_max_filter = Filter_LowPass(ctrl->dac_data_max, dac_max, max_level_shift);		// Filter DAC max value. Low values result in inaccurate vled_ad, on fast changes the filter factor will drop
	
	
	
	ctrl->dac_data = dac_max_filter;//DAC_MIN + ((dac_max_filter * (DAC_MAX - DAC_MIN))/DAC_MAX); // Calculate the current DAC value, starting from DAC_MIN linear to DAC_MAX (dac_max_filter starts from 0)
	
	if(ctrl->dac_data > DAC_MAX)						// Check if value was not a negative value or outside the 12-bit DAC range
		ctrl->dac_data = DAC_MIN;						// Set to the minimum value where DAC and AC can operate OK
	else if(ctrl->dac_data < DAC_MIN)
		ctrl->dac_data = DAC_MIN;

	*/
	
	uint32_t dac_scale_max = 0;
	uint32_t par1 = (mA * CONST_E) + (toff * vled_ad * CONST_F);
	uint32_t par2 = ((vin_ad - vled_ad) * CONST_G) + CONST_H;
	if(par1 > par2)
		dac_scale_max = (par1 - par2) >> 16;
	
	uint16_t dac_scale_max_filter = Filter_LowPass(ctrl->dac_scale_max, dac_scale_max, ctrl->filter_shift);
	ctrl->dac_scale_max = Stedown_DacLimit(dac_scale_max_filter);
	
	uint32_t scale = ((dac_scale_max_filter * ctrl->level_curve16) >> 16);
	// Make ramp up from DAC_MIN to calculated DAC scale max
	ctrl->dac_data = DAC_MIN + ((scale * (ctrl->dac_scale_max - DAC_MIN))/ctrl->dac_scale_max); // Calculate the current DAC value, starting from DAC_MIN linear to DAC_MAX (dac_max_filter starts from 0)
	
	ctrl->dac_data = Stedown_DacLimit(ctrl->dac_data);

	//if(ctrl->channel == 1)
	//	trace_var = ctrl->dac_data;
	
#if DAC_MIN > DAC_MIN_VIRTUAL
	ctrl->dac_data_real = dac_scale_max_filter;// DAC_MIN_VIRTUAL + ((dac_max_filter * (DAC_MAX - DAC_MIN_VIRTUAL))/DAC_MAX);
	if(ctrl->dac_data_real < DAC_MIN_VIRTUAL)
		ctrl->dac_data_real = DAC_MIN_VIRTUAL;
#endif

	uint32_t dac_power = ((((POWER_MAX * CONST_K) / vled_ad) << 16) / ctrl->level_curve16) + ((((toff_set + CONST_D) * vled_ad * CONST_L) - ((vin_ad - vled_ad) * CONST_M)) >> 16) - CONST_N;	
	if(dac_power < DAC_MIN)
		dac_power = DAC_MIN;
	else if(dac_power > DAC_MAX)
		dac_power = DAC_MAX;
	
	if(ctrl->dac_data > dac_power)
	{
		ctrl->dac_data = dac_power;
		State_SetWarning(ctrl->channel == 1 ? STATE_WARNING_POWER1_HIGH : STATE_WARNING_POWER2_HIGH);
	}
	else
		State_ClearWarning(ctrl->channel == 1 ? STATE_WARNING_POWER1_HIGH : STATE_WARNING_POWER2_HIGH);
	
	//ctrl->dac_data_max = dac_max;
	
	uint16_t power = (((((((ctrl->dac_data * CONST_O) + CONST_P + ((vin_ad - vled_ad) * CONST_Q) - ((toff_set + CONST_D) * vled_ad * CONST_U)) >> 12) * vled_ad) >> 8) * ctrl->level_curve16) >> 16);
	ctrl->power = power;	
	
	ctrl->stepdown_offtime = toff_set;
	
	if(ctrl->channel == 1)
	{
		TCX_TIMER_CH1->COUNT16.CCBUF[1].reg = ctrl->stepdown_offtime;
		TCX_TIMER_CH1->COUNT16.CCBUF[0].reg = ctrl->stepdown_offtime + 3;
	}
	else if(ctrl->channel == 2)
	{
		TCX_TIMER_CH2->COUNT16.CCBUF[1].reg = ctrl->stepdown_offtime;
		TCX_TIMER_CH2->COUNT16.CCBUF[0].reg = ctrl->stepdown_offtime + 3;
	}
	else if(ctrl->channel == 3)
	{
		TCX_TIMER_CH3->COUNT16.CCBUF[1].reg = ctrl->stepdown_offtime;
		TCX_TIMER_CH3->COUNT16.CCBUF[0].reg = ctrl->stepdown_offtime + 3;
	}
	else
	{
		TCX_TIMER_CH4->COUNT16.CCBUF[1].reg = ctrl->stepdown_offtime;
		TCX_TIMER_CH4->COUNT16.CCBUF[0].reg = ctrl->stepdown_offtime + 3;
	}
	
	ctrl->mA = mA_org;
}

void Stepdown_SetCurrentCh1(uint8_t chan_idx)
{
	Stepdown_SetCurrentCalc(StepDown_GetCurrent(), &stepdown_ctrl_chx[0], chan_idx);
}

void Stepdown_SetCurrentCh2(uint8_t chan_idx)
{
	Stepdown_SetCurrentCalc(StepDown_GetCurrent(), &stepdown_ctrl_chx[1], chan_idx);
}

void Stepdown_SetCurrentCh3(uint8_t chan_idx)
{
	Stepdown_SetCurrentCalc(StepDown_GetCurrent(), &stepdown_ctrl_chx[2], chan_idx);
}

void Stepdown_SetCurrentCh4(uint8_t chan_idx)
{
	Stepdown_SetCurrentCalc(StepDown_GetCurrent(), &stepdown_ctrl_chx[3], chan_idx);
}

void Stepdown_SetCurrent()
{
	Stepdown_SetCurrentCh1(0);
	Stepdown_SetCurrentCh2(1);
	Stepdown_SetCurrentCh3(2);
	Stepdown_SetCurrentCh4(3);
}

uint32_t StepDown_GetPower()
{
	return stepdown_power;
}

void StepDown_PowerCheck()
{
	stepdown_power = 0;
	for(uint8_t i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		stepdown_power += stepdown_ctrl_chx[i].power;			// Add all channels for total power
	}
}

bool StepDown_SleepCheck()
{
	for(int i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		if(stepdown_ctrl_chx[0].level > 0)
			return false;
	}
	return true;
}

uint8_t StepDown_GetDutyPercentage(uint8_t chan_idx)
{
	if(chan_idx >= MAX_DIM_CHANNELS)
		return 0;
	
	uint32_t lvl = stepdown_ctrl_chx[chan_idx].level_curve;
	if(lvl == 0)	
		return 0;
	uint8_t perc = ((lvl + 1) * 100) / 0x1000000;
	if(perc == 0)
		return 1;			// Return 1% when its on but very low
		
	return perc;
}

void StepDown_Logging()
{
	if(!StepDown_SleepCheck())		// Don't store anything when one of the channels is on
		return;
	
	TickType_t ticks = xTaskGetTickCount();
	bool many_changes = (appLogData->change_count > 3);
	if(many_changes || ((ticks - last_log_ticks) >= (12*60*60*1000UL)))		// Every half a day store current log content, or when more than 6 changes are reported
	{
		AppLog_Set();				// Set can be seen if level is not 0!!
		if(!IziPlus_Module_IsIdentifying())	
			State_SetAttentionInfo(COLOR_GREEN, COLOR_AQUA, 2, 1);
		last_log_ticks = xTaskGetTickCount();
	}
}

bool do_ff = false;
uint8_t last_freq = 0xAA;
uint8_t stimer_prs = 0;
uint8_t err_report = 0;
uint8_t sectimer_prs = 0;
uint32_t sec_after_pup = 0;		// 2 sec power-up
static uint16_t stepdown_timer_prs = 0, stepdown_timer_prs2 = 0;

void StepDown_Timer()
{
	if(time_after_pup < 0xFFFF)
		++time_after_pup;
	
	if(++sectimer_prs >= (1000/STEPDOWN_TIMEROS_TICKS))
	{
		sectimer_prs = 0;
		stepdown_force_update = 3;
		sec_after_pup++;
	}
	
	IziInput_10ms();
	
	if(++stimer_prs >= 8)
	{
		stimer_prs = 0;
	
		if((err_report == 1 || stepdown_force_update > 0) && (stepdown_debug_level > 0))
		{
			OS_TRACE_ERROR("{\"Cmd\":\"ERR\",\"Error\":%d,\"ErrStr\":\"%s\",\"Warning\":%d,\"WrnStr\":\"%s\"}\r\n", 
				State_GetErrors(), State_GetHighestErrorText(), State_GetWarnings(), State_GetHighestWarningText());	
			if(stepdown_force_update > 0)
				stepdown_force_update--;
		}
		if(++err_report > 16)
			err_report = 0;
	}
	boot_data.last_code = 'Q';
	if(++stepdown_timer_prs >= (1000/STEPDOWN_TIMEROS_TICKS))			// Every second
	{
		stepdown_timer_prs = 0;
	}
	else if(stepdown_timer_prs == 500/STEPDOWN_TIMEROS_TICKS)
	{
		REPORT_STACK(TASK_STEPDOWN_STACK_SIZE, TASK_STEPDOWN);
	}
	else if(stepdown_timer_prs == 250/STEPDOWN_TIMEROS_TICKS)
	{
		if(++stepdown_timer_prs2 >= 10 && !StepDown_AnyChannelActive())
		{
			//IziPlus_Module_CheckDefinitions();				// Check if definitions of emitter (or local) are still valid
			stepdown_timer_prs2 = 0;						// Do every 10 seconds so error will be shortly visible
		}
	}
}

bool stepdown_started = false, stepdown_started_first = false;
void StepDown_Start()
{
	uint8_t sel = stepdown_ctrl_chx[0].timing_sel > 0 ? 0 : 1;		// The buffer that was just filled
	
	StepDown_InitPwmChannel1();
	StepDown_InitPwmChannel2();
	StepDown_InitPwmChannel3();
	StepDown_InitPwmChannel4();
	
	TCCX_TIMER_EN_CH1->PER.reg = stepdown_ctrl_chx[0].per[sel][0];
	TCCX_TIMER_EN_CH1->CC[0].reg = stepdown_ctrl_chx[0].cc0[sel][0];
	TCCX_TIMER_EN_CH1->CC[DIM1_WO].reg = stepdown_ctrl_chx[0].ccx[sel][0];
	stepdown_ctrl_chx[0].timing_sel = sel;
	stepdown_ctrl_chx[0].cnt = 1;
	stepdown_ctrl_chx[0].switch_sel = -1;
	TCCX_TIMER_EN_CH1->COUNT.reg = (uint16_t)stepdown_ctrl_chx[0].per[sel][0] - 1;
	
	TCCX_TIMER_EN_CH2->PER.reg = stepdown_ctrl_chx[1].per[sel][0];
	TCCX_TIMER_EN_CH2->CC[0].reg = stepdown_ctrl_chx[1].cc0[sel][0];
	TCCX_TIMER_EN_CH2->CC[DIM2_WO].reg = stepdown_ctrl_chx[1].ccx[sel][0];
	stepdown_ctrl_chx[1].timing_sel = sel;
	stepdown_ctrl_chx[1].cnt = 1;
	stepdown_ctrl_chx[1].switch_sel = -1;
	TCCX_TIMER_EN_CH2->COUNT.reg = (uint16_t)stepdown_ctrl_chx[1].per[sel][0] - (((GCLK_GEN3_FREQ / PWM_END_FREQ) / 4) * 1);
		
	TCCX_TIMER_EN_CH3->PER.reg = stepdown_ctrl_chx[2].per[sel][0];
	TCCX_TIMER_EN_CH3->CC[0].reg = stepdown_ctrl_chx[2].cc0[sel][0];
	TCCX_TIMER_EN_CH3->CC[DIM3_WO].reg = stepdown_ctrl_chx[2].ccx[sel][0];
	stepdown_ctrl_chx[2].timing_sel = sel;
	stepdown_ctrl_chx[2].cnt = 1;
	stepdown_ctrl_chx[2].switch_sel = -1;
	TCCX_TIMER_EN_CH3->COUNT.reg = (uint16_t)stepdown_ctrl_chx[2].per[sel][0] - (((GCLK_GEN3_FREQ / PWM_END_FREQ) / 4) * 2);
	
	TCCX_TIMER_EN_CH4->PER.reg = stepdown_ctrl_chx[3].per[sel][0];
	TCCX_TIMER_EN_CH4->CC[0].reg = stepdown_ctrl_chx[3].cc0[sel][0];
	TCCX_TIMER_EN_CH4->CC[DIM4_WO].reg = stepdown_ctrl_chx[3].ccx[sel][0];
	stepdown_ctrl_chx[3].timing_sel = sel;
	stepdown_ctrl_chx[3].cnt = 1;
	stepdown_ctrl_chx[3].switch_sel = -1;
	TCCX_TIMER_EN_CH4->COUNT.reg = (uint16_t)stepdown_ctrl_chx[3].per[sel][0] - (((GCLK_GEN3_FREQ / PWM_END_FREQ) / 4) * 3);
	
	__disable_irq();
	
	TCCX_TIMER_EN_CH1->CTRLA.bit.ENABLE = true;
	TCCX_TIMER_EN_CH2->CTRLA.bit.ENABLE = true;
	TCCX_TIMER_EN_CH3->CTRLA.bit.ENABLE = true;
	TCCX_TIMER_EN_CH4->CTRLA.bit.ENABLE = true;
	
	while (TCCX_TIMER_EN_CH1->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
	while (TCCX_TIMER_EN_CH2->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
	while (TCCX_TIMER_EN_CH3->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
	while (TCCX_TIMER_EN_CH4->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
	
	/*TCCX_TIMER_EN_CH1->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	while (TCCX_TIMER_EN_CH1->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB);
	TCCX_TIMER_EN_CH2->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	while (TCCX_TIMER_EN_CH2->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB);
	TCCX_TIMER_EN_CH3->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	while (TCCX_TIMER_EN_CH3->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB);
	TCCX_TIMER_EN_CH4->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	while (TCCX_TIMER_EN_CH4->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB);
	*/
	__enable_irq();
	
	// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
	CCL_LUT_CH1.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	gpio_set_pin_function(DIM1, DIM1_FUNC);
	
	// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
	CCL_LUT_CH2.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	gpio_set_pin_function(DIM2, DIM2_FUNC);
	
	// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
	CCL_LUT_CH3.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	gpio_set_pin_function(DIM3, DIM3_FUNC);
	
	CCL_LUT_CH4.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
	gpio_set_pin_function(DIM4, DIM4_FUNC);
	
	stepdown_started = true;
	stepdown_started_first = true;
}

void StepDown_Stop()
{
	/*TCCX_TIMER_EN_CH1->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
	TCCX_TIMER_EN_CH2->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
	TCCX_TIMER_EN_CH3->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
	TCCX_TIMER_EN_CH4->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
	*/
	TCCX_TIMER_EN_CH1->CTRLA.bit.ENABLE = false;
	TCCX_TIMER_EN_CH2->CTRLA.bit.ENABLE = false;
	TCCX_TIMER_EN_CH3->CTRLA.bit.ENABLE = false;
	TCCX_TIMER_EN_CH4->CTRLA.bit.ENABLE = false;
	
	stepdown_started = false;
}

void StepDown_ChannelEnable_Ch1(bool start, uint8_t chan_idx)
{
	uint8_t sel = stepdown_ctrl_chx[chan_idx].timing_sel > 0 ? 0 : 1;		// The buffer that was just filled
	if(start && stepdown_ctrl_chx[chan_idx].per[sel][0] > 30)
	{
		//stepdown_ctrl_chx[chan_idx].switch_sel = -1;
		//stepdown_ctrl_chx[chan_idx].timing_sel = sel;
		
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH1.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		gpio_set_pin_function(DIM1, DIM1_FUNC);
	}
	else
	{
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH1.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		
		PORT->Group[DIM1_PORT].OUTSET.reg = 0x01 << DIM1_PIN;
		gpio_set_pin_direction(DIM1, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(DIM1, GPIO_PIN_FUNCTION_OFF);

		gpio_set_pin_direction(SW1, GPIO_DIRECTION_OUT);
		PORT->Group[SW1_PORT].OUTCLR.reg = 0x01 << SW1_PIN;				// Set Low = off
	}	
}

void StepDown_ChannelEnable_Ch2(bool start, uint8_t chan_idx)
{
	uint8_t sel = stepdown_ctrl_chx[chan_idx].timing_sel > 0 ? 0 : 1;		// The buffer that was just filled
	if(start && stepdown_ctrl_chx[chan_idx].per[sel][0] > 30)
	{
		//stepdown_ctrl_chx[chan_idx].switch_sel = -1;
		//stepdown_ctrl_chx[chan_idx].timing_sel = sel;
		
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH2.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		
		gpio_set_pin_function(DIM2, DIM2_FUNC);
	}
	else
	{
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH2.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		
		PORT->Group[DIM2_PORT].OUTSET.reg = 0x01 << DIM2_PIN;
		gpio_set_pin_direction(DIM2, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(DIM2, GPIO_PIN_FUNCTION_OFF);

		gpio_set_pin_direction(SW2, GPIO_DIRECTION_OUT);
		PORT->Group[SW2_PORT].OUTCLR.reg = 0x01 << SW2_PIN;				// Set Low = off
	}
}

void StepDown_ChannelEnable_Ch3(bool start, uint8_t chan_idx)
{
	uint8_t sel = stepdown_ctrl_chx[chan_idx].timing_sel > 0 ? 0 : 1;		// The buffer that was just filled
	if(start && stepdown_ctrl_chx[chan_idx].per[sel][0] > 30)
	{
		//stepdown_ctrl_chx[chan_idx].switch_sel = -1;
		//stepdown_ctrl_chx[chan_idx].timing_sel = sel;
		
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH3.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		
		gpio_set_pin_function(DIM3, DIM3_FUNC);
	}
	else
	{
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH3.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		
		PORT->Group[DIM3_PORT].OUTSET.reg = 0x01 << DIM3_PIN;
		gpio_set_pin_direction(DIM3, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(DIM3, GPIO_PIN_FUNCTION_OFF);

		gpio_set_pin_direction(SW3, GPIO_DIRECTION_OUT);
		PORT->Group[SW3_PORT].OUTCLR.reg = 0x01 << SW3_PIN;				// Set Low = off
	}
}

void StepDown_ChannelEnable_Ch4(bool start, uint8_t chan_idx)
{
	uint8_t sel = stepdown_ctrl_chx[chan_idx].timing_sel > 0 ? 0 : 1;		// The buffer that was just filled
	if(start && stepdown_ctrl_chx[chan_idx].per[sel][0] > 30)
	{
		//stepdown_ctrl_chx[chan_idx].switch_sel = -1;
		//stepdown_ctrl_chx[chan_idx].timing_sel = sel;
		
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH4.reg = CCL_LUTCTRL_ENABLE | CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		//CCL->CTRL.reg = CCL_CTRL_ENABLE;
		
		gpio_set_pin_function(DIM4, DIM4_FUNC);
	}
	else
	{
		// This probably does not work due to a errata that the enable protect of the register does not work OK, the complete CCL must be disabled: CCL->CTRL.reg = 0;
		CCL_LUT_CH4.reg = CCL_LUTCTRL_INSEL0_TCC | CCL_LUTCTRL_INSEL1_TC | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_TRUTH(1 << 0);
		//CCL->CTRL.reg = CCL_CTRL_ENABLE;
		
		PORT->Group[DIM4_PORT].OUTSET.reg = 0x01 << DIM4_PIN;
		gpio_set_pin_direction(DIM4, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(DIM4, GPIO_PIN_FUNCTION_OFF);

		gpio_set_pin_direction(SW4, GPIO_DIRECTION_OUT);
		PORT->Group[SW4_PORT].OUTCLR.reg = 0x01 << SW4_PIN;				// Set Low = off
	}
}

// Ramp must go through 0,0 for more stable frequency shift!!
#define PWM_END_LEVEL		0x400000				// Frequency ramp up end (0x200000 somehow makes the best fade!!!)
#define PWM_START_LEVEL		((PWM_END_LEVEL * PWM_START_FREQ) / PWM_END_FREQ)				// Frequency ramp up start, make it a linear line that goes through 0,0 so when the frequency is change the pulse width stays the same

static uint16_t StepDown_GetFrequency(uint32_t max_level)
{
	uint16_t freq_new;
	if(max_level == 0)
		freq_new = 0;
	else if(max_level <= PWM_START_LEVEL)							// Determine frequency that belongs to current PWM level (between 1kHz and 22kHz)
		freq_new = PWM_START_FREQ;
	else if(max_level <= PWM_END_LEVEL)
		freq_new = PWM_START_FREQ + ((uint64_t)((uint64_t)(max_level - PWM_START_LEVEL) * (uint64_t)(PWM_END_FREQ - PWM_START_FREQ)) / (PWM_END_LEVEL - PWM_START_LEVEL));
	else
		freq_new = PWM_END_FREQ;
	
	return freq_new;
}

#define RANDOM_OFF_LVL24		0x400
#define RANDOM_CLIP_LVL24		0x40000
#define RANDOM_FULL_LVL24		0x100000			// At this 24-bit level the random is full
#define RANDOM_DOWN_LVL24		0x800000
#define RANDOM_OFFDOWN_LVL24	0xB00000

static int8_t first_active_chx = -1;
 
//const int random_table[MAX_TABLE] = { 23845, 437, 59234, 16789, 39482, 52301, 1298, 45670, 30876, 15043, 60789, 3987, 48921, 12456, 55021, 32876, 9943, 63478, 24987, 57034, 944, 34567, 22111, 59123 };
const unsigned short random_table2[MAX_TABLE] = { 1726, 768, 1109, 419, 1602, 780, 1975, 261, 1376, 62, 1636, 699, 1575, 246, 1314, 79 };
	
//const unsigned short random_table[MAX_TABLE] = { 0, 150, 300, 450, 600, 750, 900, 1050, 1200, 1350, 1500, 1650, 1500, 1350, 1200, 1050, 900, 750, 600, 450, 300, 150, 75, 0 };			// Best for noise
const uint16_t random_table[MAX_TABLE] = {
	0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 3500, 3000, 2500, 2000, 1500, 1000, 500
};
		
static void StepDown_SetPwm(volatile stepdown_control_t *ctrl, uint8_t chan_idx)
{
	int32_t level_pwm_chx_t = ctrl->pwm_new;			// Store calculated value
	
	uint8_t sel = ctrl->timing_sel > 0 ? 0 : 1;
	if(ctrl->level_curve == 0 || StepDown_IsClosed())	// Check if not in safety situation, if so, set to 0
	{
		ctrl->pwm_new = 0;
		ctrl->channel_enable(false, chan_idx);
	}
	else
	{
		if(chan_idx == 0)
			stepdown_ctrl_chx[0].test = ctrl->level_curve >> 16;
		
		ctrl->freq_new = StepDown_GetFrequency(ctrl->level_curve);	// Get the frequency for the channel (in case of WD and TW the master value will be used, this is not good for the resolution, but ok for the sound as in annoying hearable frequencies, in WD white and blue will start with 22kHz, beware that the DAC also used the master)
		
		for(int i = 0; i < MAX_TABLE; i++)
		{
			uint32_t random = (random_table[i]);// * 0.5) + (rand_sync_read32(&RAND_0) & 0x02FF);		// Using table is better for sound and is more predictable
			uint32_t random_offset;
				
			// Must use level_curve_master, because it is only calculated once for all 4 channel to stay in sync
			if(ctrl->level_curve <= RANDOM_DOWN_LVL24)
			{
				uint32_t max_level_random = ctrl->level_curve;
				if(ctrl->level_curve < RANDOM_CLIP_LVL24)
					max_level_random = RANDOM_CLIP_LVL24;
				random_offset = ((max_level_random >= RANDOM_FULL_LVL24) ? random : ((random * (max_level_random >> 4)) / (RANDOM_FULL_LVL24 >> 4)));  // Reduced randomness at lower levels (better for real low levels, not enough resolution and real low values will flicker), realy low is better with small random to keep color more stable (>> 4 is to avoid overflow)
			}
			else if(ctrl->level_curve >= (RANDOM_OFFDOWN_LVL24 - ((RANDOM_OFFDOWN_LVL24 - RANDOM_DOWN_LVL24) / 4)))
			{
				random_offset = random / 4;
			}
			else
				random_offset = (ctrl->level_curve < RANDOM_OFFDOWN_LVL24) ? ((random * ((RANDOM_OFFDOWN_LVL24 - ctrl->level_curve) >> 4)) / ((RANDOM_OFFDOWN_LVL24 - RANDOM_DOWN_LVL24) >> 4)) : 0;  // Reduced randomness at lower levels (current random table can cause tone near 11kHz) (>> 4 is to avoid overflow)
								
			//if(appConfig->config[APPCFG_FILTERMODE] == CONFIG_FILTER_SLOW || appConfig->config[APPCFG_FILTERMODE] == CONFIG_FILTER_FAST)
			//	random_offset = 0;
			uint32_t freq_shifted = ctrl->freq_new + random_offset;
			//stepdown_ctrl_chx[chan_idx].test = random_offset;
			
			ctrl->per[sel][i] = ((ctrl->tcc_freq) / freq_shifted);
				
			//if(i == 8)
			//	stepdown_ctrl_chx[chan_idx].test = random_offset;
						
			uint32_t pwm_new = ((uint64_t)((uint64_t)ctrl->per[sel][i] * (uint64_t)ctrl->level_curve) / 0x1000000UL);
			pwm_new += 3;//chan_idx == 1 ? 0 : 8;						// Add 6 ticks for compensation of latency DIM to output
			if(pwm_new > ctrl->per[sel][i])
				pwm_new = ctrl->per[sel][i] - 3;
			else if(pwm_new == 0)				// Pwm may not be the same as period reg
				pwm_new = 1;
			
#if DAC_MIN_VIRTUAL < DAC_MIN			
			if(ctrl->dac_data_real < ctrl->dac_data)
				pwm_new = (pwm_new * ctrl->dac_data_real)/ctrl->dac_data;			// Compensate diff that DAC couldn't do
#endif
				
			pwm_new = ctrl->per[sel][i] - pwm_new;
							
			// Precharge
			uint32_t shunt_offset = ((((((pwm_new * CONST_I) >> 8) * ctrl->dac_data) >> 16) + CONST_J) * ctrl->mA) / ctrl->vin_ad;
		#ifdef CC2_RANDOM_OFF
			if(shunt_offset > 32)
				shunt_offset -= 32;
			shunt_offset += (rand_sync_read32(&RAND_0) & 0x3F);
		#endif
			
			if(pwm_new == 0)
				pwm_new++;
				
			if(pwm_new > shunt_offset)
				ctrl->cc0[sel][i] = pwm_new - (shunt_offset);
			else
				ctrl->cc0[sel][i] = 1;
				
			ctrl->ccx[sel][i] = pwm_new;
		}
		
		ctrl->period = ctrl->per[sel][0];
		ctrl->pwm_new = ctrl->ccx[sel][0];
		
		if(level_pwm_chx_t == 0)
		{
			if(stepdown_started)
				ctrl->channel_enable(true, chan_idx);
		}
		if(first_active_chx < 0)	
			first_active_chx = chan_idx;
	}
}

#define MAX_CURVES	3
const uint16_t dmxTable[MAX_CURVES][257] =
{
	{   // Power 1.5
		0,	2,	18,	47,	86,	131,	182,	239,	301,	368,	439,	513,	592,	674,	760,	850,
		942,	1038,	1136,	1238,	1342,	1449,	1559,	1672,	1787,	1905,	2025,	2148,	2273,	2400,	2530,	2662,
		2796,	2932,	3070,	3211,	3354,	3498,	3645,	3794,	3944,	4097,	4251,	4408,	4566,	4726,	4888,	5052,
		5218,	5385,	5554,	5725,	5898,	6072,	6248,	6425,	6605,	6786,	6968,	7152,	7338,	7525,	7714,	7905,
		8097,	8290,	8485,	8682,	8880,	9079,	9280,	9483,	9686,	9892,	10099,	10307,	10516,	10727,	10940,	11153,
		11369,	11585,	11803,	12022,	12243,	12465,	12688,	12912,	13138,	13365,	13594,	13824,	14055,	14287,	14520,	14755,
		14991,	15229,	15467,	15707,	15948,	16190,	16434,	16678,	16924,	17171,	17419,	17669,	17919,	18171,	18424,	18678,
		18933,	19190,	19447,	19706,	19966,	20227,	20489,	20752,	21017,	21282,	21549,	21816,	22085,	22355,	22626,	22898,
		23171,	23445,	23720,	23997,	24274,	24553,	24832,	25113,	25394,	25677,	25961,	26245,	26531,	26818,	27106,	27395,
		27685,	27975,	28267,	28560,	28854,	29149,	29445,	29742,	30040,	30339,	30639,	30939,	31241,	31544,	31848,	32153,
		32458,	32765,	33073,	33381,	33691,	34001,	34313,	34625,	34938,	35253,	35568,	35884,	36201,	36519,	36838,	37158,
		37479,	37800,	38123,	38446,	38771,	39096,	39422,	39750,	40078,	40407,	40736,	41067,	41399,	41731,	42065,	42399,
		42734,	43070,	43407,	43745,	44084,	44423,	44763,	45105,	45447,	45790,	46134,	46478,	46824,	47170,	47518,	47866,
		48215,	48564,	48915,	49266,	49619,	49972,	50326,	50681,	51036,	51393,	51750,	52108,	52467,	52827,	53188,	53549,
		53911,	54274,	54638,	55003,	55368,	55734,	56101,	56469,	56838,	57207,	57578,	57949,	58321,	58693,	59067,	59441,
		59816,	60192,	60568,	60946,	61324,	61703,	62082,	62463,	62844,	63226,	63609,	63993,	64377,	64762,	65148,	65535,
		65535
	},
	{   // Power 2.0
		0,	2,	3,	6,	11,	18,	27,	38,	51,	67,	84,	103,	124,	148,	173,	201,
		230,	262,	295,	331,	368,	408,	449,	493,	539,	587,	636,	688,	742,	798,	856,	916,
		978,	1042,	1108,	1176,	1246,	1318,	1392,	1468,	1546,	1627,	1709,	1793,	1880,	1968,	2058,	2151,
		2245,	2342,	2440,	2541,	2644,	2748,	2855,	2963,	3074,	3187,	3302,	3419,	3537,	3658,	3781,	3906,
		4033,	4162,	4293,	4426,	4561,	4698,	4838,	4979,	5122,	5267,	5415,	5564,	5715,	5869,	6024,	6181,
		6341,	6502,	6666,	6831,	6999,	7169,	7340,	7514,	7690,	7868,	8047,	8229,	8413,	8599,	8787,	8977,
		9169,	9363,	9559,	9757,	9957,	10159,	10363,	10570,	10778,	10988,	11200,	11415,	11631,	11849,	12070,	12292,
		12517,	12743,	12972,	13202,	13435,	13670,	13906,	14145,	14386,	14628,	14873,	15120,	15369,	15620,	15873,	16128,
		16385,	16644,	16905,	17168,	17433,	17700,	17969,	18241,	18514,	18789,	19066,	19346,	19627,	19910,	20196,	20483,
		20773,	21064,	21358,	21654,	21951,	22251,	22552,	22856,	23162,	23470,	23780,	24091,	24405,	24721,	25039,	25359,
		25681,	26005,	26331,	26659,	26989,	27321,	27656,	27992,	28330,	28670,	29013,	29357,	29703,	30052,	30402,	30755,
		31109,	31466,	31824,	32185,	32548,	32912,	33279,	33648,	34018,	34391,	34766,	35143,	35522,	35903,	36286,	36671,
		37058,	37447,	37838,	38231,	38626,	39023,	39422,	39823,	40227,	40632,	41039,	41449,	41860,	42274,	42689,	43106,
		43526,	43947,	44371,	44797,	45224,	45654,	46086,	46519,	46955,	47393,	47833,	48275,	48719,	49164,	49612,	50062,
		50514,	50968,	51425,	51883,	52343,	52805,	53269,	53735,	54204,	54674,	55146,	55621,	56097,	56575,	57056,	57538,
		58023,	58509,	58998,	59489,	59981,	60476,	60973,	61471,	61972,	62475,	62980,	63487,	63996,	64507,	65020,	65535,
		65535
	},
	{   // Power 2.5
		0,	2,	3,	4,	5,	6,	7,	8,	10,	13,	17,	22,	27,	33,	40,	48,
		57,	67,	77,	89,	102,	116,	130,	146,	163,	181,	201,	221,	243,	266,	290,	316,
		343,	371,	400,	431,	463,	497,	532,	569,	607,	646,	688,	730,	774,	820,	867,	916,
		967,	1019,	1073,	1128,	1185,	1244,	1305,	1367,	1431,	1497,	1565,	1634,	1706,	1779,	1854,	1931,
		2009,	2090,	2172,	2257,	2343,	2432,	2522,	2614,	2709,	2805,	2903,	3004,	3106,	3211,	3317,	3426,
		3537,	3650,	3765,	3882,	4002,	4123,	4247,	4373,	4501,	4632,	4764,	4899,	5036,	5176,	5317,	5462,
		5608,	5757,	5908,	6061,	6217,	6375,	6536,	6698,	6864,	7032,	7202,	7374,	7550,	7727,	7907,	8090,
		8275,	8462,	8653,	8845,	9040,	9238,	9439,	9642,	9847,	10055,	10266,	10479,	10695,	10914,	11135,	11360,
		11586,	11816,	12048,	12283,	12520,	12760,	13003,	13249,	13498,	13749,	14003,	14260,	14520,	14782,	15048,	15316,
		15587,	15861,	16137,	16417,	16700,	16985,	17273,	17565,	17859,	18156,	18456,	18759,	19065,	19374,	19686,	20001,
		20319,	20640,	20964,	21291,	21621,	21954,	22290,	22629,	22972,	23317,	23666,	24017,	24372,	24730,	25091,	25455,
		25822,	26193,	26566,	26943,	27323,	27706,	28093,	28482,	28875,	29271,	29671,	30073,	30479,	30888,	31300,	31716,
		32135,	32557,	32983,	33412,	33844,	34280,	34718,	35161,	35606,	36055,	36508,	36963,	37423,	37885,	38351,	38820,
		39293,	39770,	40249,	40732,	41219,	41709,	42203,	42700,	43200,	43704,	44212,	44723,	45238,	45756,	46278,	46803,
		47332,	47864,	48400,	48940,	49483,	50029,	50580,	51134,	51691,	52253,	52818,	53386,	53958,	54534,	55114,	55697,
		56284,	56874,	57468,	58066,	58668,	59274,	59883,	60496,	61112,	61733,	62357,	62985,	63617,	64252,	64891,	65535,
		65535

	}
};


static void StepDown_SetControl(uint16_t level_chx, volatile  stepdown_control_t *ctrl, uint8_t chan_idx)
{
	// Filter
	int32_t level_old = ctrl->level_filter;
	uint8_t filter_mode = CONFIG_PAR_FILTER(appConfig->config);
	uint8_t curve_select = appConfig->config[APPCFG_CURVE];
	
	int delta = abs((int)(level_chx >> 8) - (int)(ctrl->level >> 8));
	if(delta == 0)
	{
		if(ctrl->no_change_cnt < 32)
			ctrl->no_change_cnt++;
	}
	else
		ctrl->no_change_cnt = 0;
	
	int max = (ctrl->no_change_cnt / 16) + 1;			// If there have been no changes for more than 16 ticks = 128ms, 2 extra loops can be done to avoid a real slow step when dmx is only set 1 raw step
	for(int i = 0; i < max; i++)						// There is a small delay, because it does work when a slow fade is done
	{
		// Do filtering
		if(filter_mode == CONFIG_FILTER_DYNAMIC || IziPlus_Module_IsIdentifying())		// Dynamic mode, checks the change speed in level, if large steps are made in dmx, the filter will be made faster, so it is more responsive
		{
			int speed = (delta / 16);
			if(speed > 3)
				speed = 3;
			
			if(((speed >= ctrl->filter_fast_speed) || (ctrl->filter_fast_time == 0)) && time_after_pup > 350)	// No fast filter just after power-up
			{
				// Is it the highest or same speed, set others too to the same speed for (almost) equal fade
				for(int c = 0; c < MAX_DIM_CHANNELS; c++)
				{
					stepdown_ctrl_chx[c].filter_fast_time = 32;					// Use the speed for 32 ticks = 256ms
					stepdown_ctrl_chx[c].filter_fast_speed = speed;
				}
			}
			ctrl->filter_shift = 4 - ctrl->filter_fast_speed;
			ctrl->level_filter = ((level_chx - ctrl->level_filter) >> ctrl->filter_shift) + ctrl->level_filter;		// Low pass filter
			
			if(ctrl->filter_fast_time > 0)
				ctrl->filter_fast_time--;
			stepdown_task_fast = false;
		}
		else if(filter_mode == CONFIG_FILTER_FAST && (time_after_pup > 350))		// Always fast filter (except just after power-up)
		{
			ctrl->level_filter = ((level_chx - ctrl->level_filter) / 16) + ctrl->level_filter;
			ctrl->filter_fast_time = 0;
			ctrl->filter_shift = 4;
			stepdown_task_fast = true;
		}
		else if(filter_mode == CONFIG_FILTER_SLOW)		// Always slow filter
		{
			ctrl->level_filter = ((level_chx - ctrl->level_filter) / 64) + ctrl->level_filter;
			ctrl->filter_fast_time = 0;
			ctrl->filter_shift = 6;
			stepdown_task_fast = false;
		}
		else
		{
			ctrl->level_filter = ((level_chx - ctrl->level_filter) / 16) + ctrl->level_filter;
			ctrl->filter_fast_time = 0;
			ctrl->filter_shift = 4;
			stepdown_task_fast = false;
		}
		
		if(ctrl->level_filter == level_old && level_chx != ctrl->level_filter)		// Make sure if last delta is divided to 0, that the real level is still reached in steps of 1
		{
			if(level_chx > ctrl->level_filter)
				ctrl->level_filter++;
			else
				ctrl->level_filter--;
		}
		
		ctrl->level = level_chx;
		
		uint16_t level_filter_old = ctrl->level_curve;
#ifndef MS_MEASURE
		if(curve_select > 0 && curve_select <= MAX_CURVES)
		{
			uint16_t idx  = ctrl->level_filter >> 8;        // high byte
			uint16_t frac = ctrl->level_filter & 0xFF;      // low byte (fraction)

			uint32_t base = dmxTable[curve_select - 1][idx];                  // 16-bit table entry
			uint32_t next = dmxTable[curve_select - 1][idx + 1];              // next table entry
			
			uint32_t interp = ((uint32_t)base << 8) + (uint32_t)(next - base) * frac;
			if(ctrl->level_filter == 0xFFFF)
				interp = 0xFFFFFF;
			
			// interp is now 24-bit fixed point
			ctrl->level_curve = interp;						// already has fractional precision, no need to << 8 later
		}
		else
			ctrl->level_curve = ctrl->level_filter * 256;
#else
		ctrl->level_curve = ctrl->level_filter * 256;
#endif //MS_MEASURE
		
		
		if(level_chx != ctrl->level_filter && level_filter_old == ctrl->level_curve)		// In case the loop can be executed more times and the level has not changed at the input, but the goal is still not reached, speed up
		{
			continue;			// xtra loop if available (max)
		}
		break;
	}
	ctrl->level_curve_raw = ctrl->level_curve;
	
	// Todo: TW/WD modes
	//ctrl->level_curve = IziPlus_Module_GetTable(chan_idx, ctrl->level_curve);
	//ctrl->test2 = ctrl->level_curve;
	ctrl->level_curve = IziPlus_Module_TempLimit(ctrl->level_curve);				// Check if any temperature or power is too high
	ctrl->level_curve16 = ctrl->level_curve >> 8;
	
	//if(chan_idx == 1)
	//{
		//stepdown_ctrl_chx[0].test = (stepdown_ctrl_chx[0].level_curve * 100) / stepdown_ctrl_chx[1].level_curve;
	//}
	/*if(ctrl->level_curve > 0)
		ctrl->set_current(chan_idx);												// Calculate Toff and Dac value
	else
	{
		ctrl->power = 0;															// No calculations done, set to 0 here
		ctrl->vfled = 0;															// Needed in case of a strobe in fast or dynamic mode, else the vfled will keep rising and the power calculation will overflow/fail
		ctrl->dac_data = 0;
	}*/
	ctrl->level_curve_correct_prev = ctrl->level_curve;								// Store value for next calculation of temp correction
	
	if(((stepdown_debug_level > 0) && (stepdown_debug_channel == ctrl->channel)) && ((stepdown_debug_level > 1) || (stepdown_force_update > 0)))
	{
		OS_TRACE_ERROR("{\"Cmd\":\"PWM\",\"Time\":%d,\"Nr\":%d,\"Lvl\":%d,\"LvlF\":%d,\"Pwm\":%d,\"Period\":%d,\"Toff\":%d,\"Calc\":%d,\"Freq\":%d,\"Vin\":%d,\"Dac\":%d,\"Vled\":%d,\"Current\":%d,\"ACL\":%d,\"Tst\":%d,\"Tst2\":%d,\"DacS\":%d,\"Etemp\":%d,\"Vfled\":%d}\r\n",
			xTaskGetTickCount(), ctrl->channel, level_chx, ctrl->level_curve, ctrl->pwm_new, ctrl->period, ctrl->stepdown_offtime, ctrl->level_filter,
			ctrl->freq_new, ADC_RAW_MV(ctrl->vin_ad), ctrl->dac_data, ADC_RAW_MV(ctrl->vled_ad), ctrl->mA, ctrl->vfled, ctrl->test, ctrl->test2, ctrl->dac_data, ctrl->e_temp, ADC_RAW_MV32(ctrl->vfled));
		
		counter = 0;
		if(stepdown_force_update > 0)
			stepdown_force_update--;
	}
}

uint8_t StepDown_DebugLevel()
{
	return stepdown_debug_level;
}

bool StepDown_AnyChannelActive()
{
	for(int i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		if(stepdown_ctrl_chx[i].pwm_new > 0)
			return true;
	}
	return false;
}

//static volatile popi = 1;
void StepDown_SetOutputs()
{
	uint8_t idx = 0, on_before = 0, on_after = 0;
	
	for(int i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		// Set the DAC and toff times
		if(stepdown_ctrl_chx[i].level_curve > 0)
			stepdown_ctrl_chx[i].set_current(i);						// Calculate Toff and Dac value
		else
		{
			stepdown_ctrl_chx[i].power = 0;								// No calculations done, set to 0 here
			stepdown_ctrl_chx[i].vfled = 0;								// Needed in case of a strobe in fast or dynamic mode, else the vfled will keep rising and the power calculation will overflow/fail
			stepdown_ctrl_chx[i].dac_data = 0;
		}
		
		if(stepdown_ctrl_chx[i].pwm_new > 0)
			on_before++;
		StepDown_SetPwm(&stepdown_ctrl_chx[i], i);						// Set frequency and PWM							
		if(stepdown_ctrl_chx[i].pwm_new > 0 && idx == 0)
			idx = i;
			
		if(stepdown_ctrl_chx[i].pwm_new > 0)
			on_after++;
	}
	if(on_after)			// Any on?
	{
		for(int i = 0; i < MAX_DIM_CHANNELS; i++)
		{
			if(stepdown_ctrl_chx[i].pwm_new == 0)		// But this one is off, copy period register because timer will run to keep in sync
			{
				uint8_t sel = stepdown_ctrl_chx[i].timing_sel > 0 ? 0 : 1;		// Take buffer just filled
				for(int p = 0; p < MAX_TABLE; p++)
				{
					int per = stepdown_ctrl_chx[idx].per[sel][p];
					stepdown_ctrl_chx[i].per[sel][p] = per;
					stepdown_ctrl_chx[i].ccx[sel][p] = per - 1;
					stepdown_ctrl_chx[i].cc0[sel][p] = per - 2;
				}
			}
		}
	}
	
	int8_t switch_sel = on_before ? (stepdown_ctrl_chx[idx].cnt + 2) % MAX_TABLE : -1;
	for(int i = 0; i < MAX_DIM_CHANNELS; i++)
	{
		stepdown_ctrl_chx[i].switch_sel = switch_sel;			// Set on which count buffer must be switched
	}
	
	if((on_before > 0) != (on_after > 0))		// All off or first on?
	{
		vTaskSuspendAll();  // Stop task scheduling
		if(on_after > 0)	
		{
			StepDown_Start();
			//popi = 1;
		}
		else
			StepDown_Stop();
			
		xTaskResumeAll();  // Start task scheduling
	}

	uint16_t dac_values[4];
	dac_values[0] = stepdown_ctrl_chx[2].dac_data;		// DACA = Ch3
	dac_values[1] = stepdown_ctrl_chx[1].dac_data;		// DACB = Ch2
	dac_values[2] = stepdown_ctrl_chx[3].dac_data;		// DACC = Ch4
	dac_values[3] = stepdown_ctrl_chx[0].dac_data;		// DACD = Ch1

	Mcp4728_WriteDacs(dac_values);
}


static void StepDown_Task(void *p)
{
	TickType_t task_loop_stamp = 0;
	int task_loop_time = 0;
	
	IziPlus_Module_Init();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, EIC_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_EIC_bit(MCLK);
	
	EIC->CTRLA.bit.SWRST = 1;
	while ((EIC->SYNCBUSY.reg) & EIC_SYNCBUSY_SWRST) {
	};
	EIC->CONFIG[0].reg = EIC->CONFIG[1].reg = 0;
	EIC->CONFIG[AC_EXT_IN1_EVENT_CFG].reg |= AC_EXT_IN1_EVENT_SENSE;
	EIC->CONFIG[AC_EXT_IN2_EVENT_CFG].reg |= AC_EXT_IN2_EVENT_SENSE;
	EIC->CONFIG[AC_EXT_IN3_EVENT_CFG].reg |= AC_EXT_IN3_EVENT_SENSE;
	EIC->CONFIG[AC_EXT_IN4_EVENT_CFG].reg |= AC_EXT_IN4_EVENT_SENSE;
	EIC->EVCTRL.reg = EIC_EVCTRL_EXTINTEO(1 << AC_EXT_IN1_EVENT_BIT) |  EIC_EVCTRL_EXTINTEO(1 << AC_EXT_IN2_EVENT_BIT) |  EIC_EVCTRL_EXTINTEO(1 << AC_EXT_IN3_EVENT_BIT) | EIC_EVCTRL_EXTINTEO(1 << AC_EXT_IN4_EVENT_BIT);
	//EIC->ASYNCH.reg |= (1 << AC_EXT_IN1_EVENT_BIT) | (1 << AC_EXT_IN2_EVENT_BIT) | (1 << AC_EXT_IN3_EVENT_BIT) | (1 << AC_EXT_IN4_EVENT_BIT);		// NB: Enabling this gives shit, that channels can influence each other
	EIC->DEBOUNCEN.reg |= (1 << AC_EXT_IN1_EVENT_BIT) | (1 << AC_EXT_IN2_EVENT_BIT) | (1 << AC_EXT_IN3_EVENT_BIT) | (1 << AC_EXT_IN4_EVENT_BIT);
	EIC->DPRESCALER.reg = 0;//EIC_DPRESCALER_STATES0 | EIC_DPRESCALER_STATES1;		// Use GCLK_EIC, F/2, 4 samples, -> 1/120MHz/2 * 4 = 66ns
	
	gpio_set_pin_direction(AC_EXT_IN1, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(AC_EXT_IN1, GPIO_PULL_UP);
	gpio_set_pin_function(AC_EXT_IN1, AC_EXT_IN1_FUNC);
	gpio_set_pin_direction(AC_EXT_IN2, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(AC_EXT_IN2, GPIO_PULL_UP);
	gpio_set_pin_function(AC_EXT_IN2, AC_EXT_IN2_FUNC);
	gpio_set_pin_direction(AC_EXT_IN3, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(AC_EXT_IN3, GPIO_PULL_UP);
	gpio_set_pin_function(AC_EXT_IN3, AC_EXT_IN3_FUNC);
	gpio_set_pin_direction(AC_EXT_IN4, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(AC_EXT_IN4, GPIO_PULL_UP);
	gpio_set_pin_function(AC_EXT_IN4, AC_EXT_IN4_FUNC);
	
	EIC->CTRLA.bit.ENABLE = 1;
	
	Analog_Init();
	stepdown_task_fast = false;
	
	while(true)
	{
		uint16_t delay = stepdown_task_fast ? (STEPDOWN_TIMEROS_TICKS / 4) : (STEPDOWN_TIMEROS_TICKS/2);
		vTaskDelay(delay);						// 5ms delay (total loop should be 10ms, other delay is lower)
		boot_data.last_code = 'R';
		
		if(stepdown_closed > 0)
		{
			stepdown_closed--;
			vTaskDelay(STEPDOWN_TIMEROS_TICKS - delay);
			StepDown_Timer();
			StepDown_CheckLimits();				// Check voltage
			continue;
		}
		if(!Mcp4728_InitReady())				// Mcp4728 ok?
			if(!Mcp4728_Init())					// No, try init again
				continue;						// If failed, do nothing
		
		StepDown_PowerCheck();
		StepDown_CheckLimits();
		boot_data.last_code = '&';
		for(int i = 0; i < MAX_DIM_CHANNELS; i++)
		{
			StepDown_SetControl(IziPlus_Module_GetDim(i), &stepdown_ctrl_chx[i], i);
		}
		boot_data.last_code = '*';
		StepDown_SetOutputs();
		
		uint16_t delay2 = (stepdown_task_fast ? (STEPDOWN_TIMEROS_TICKS / 2) : STEPDOWN_TIMEROS_TICKS) - delay;
		vTaskDelay(delay2);									// Split delay to keep correct frequency				// 5ms delay (split in case something takes more than 1ms (not the case at time of writing)
		
		boot_data.last_code = '$';
		if(!I2Cx_Check())
		{
			State_SetWarning(STATE_WARNING_EMITTER_COM);
			stepdown_i2c_warning_time = 200;				// Delay so warning is shortly shown
		}
		else if(stepdown_i2c_warning_time == 0)
			State_ClearWarning(STATE_WARNING_EMITTER_COM);
		else
			stepdown_i2c_warning_time--;
		
		if(!stepdown_task_fast || stepdown_task_fast_toggle)		// Execute half of the times (so timing stays 10ms)
		{
			StepDown_Timer();
			StepDown_Logging();
			IziPlus_NoComCheck();
			stepdown_task_fast_toggle = false;
		}
		else
			stepdown_task_fast_toggle = true;
		
		TickType_t task_loop_stamp_new = xTaskGetTickCount();
		if(task_loop_stamp > 0 && (!StepDown_SleepCheck()))
		{
			if((task_loop_stamp_new - task_loop_stamp) > (stepdown_task_fast ? (STEPDOWN_TIMEROS_TICKS/2) : STEPDOWN_TIMEROS_TICKS))
			{
				State_SetWarning(STATE_WARNING_TIMING_WARN);
				task_loop_time = 400;			// 4 sec
			}
			else if(task_loop_time > 0)
			{
				if(--task_loop_time == 0)
					State_ClearWarning(STATE_WARNING_TIMING_WARN);
			}
		}
		task_loop_stamp = task_loop_stamp_new;
		
		REPORT_STACK(TASK_STEPDOWN_STACK_SIZE, TASK_STEPDOWN);
	}
}
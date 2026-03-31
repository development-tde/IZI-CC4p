/*
 * analog.c
 *
 *  Created on: 9 dec. 2020
 *      Author: Milo
 */

#include "includes.h"
#include "analog.h"
#include "appconfig.h"
#include "timers.h"
#include <hpl_adc_base.h>
#include <hal_adc_async.h>
//#include "izican/izi_can_slave.h"
#include "izilink/iziplus_driver.h"
#include "peripheral_clk_config.h"
#include "hpl_tcc_config.h"
#include "at30tse.h"
#include "stepdown.h"
#include "iziplus_module_def.h"

#define ADC_CLK_APBXMASK			((Mclk *)MCLK)->APBDMASK.bit.ADC1_
#define ADC_CLK_GCLK_ID				41 //(40 = ADC0)			// Check PCHCTRL in datasheet for index of peripheral
#define ADC_INT0					ADC1_0_Handler
#define ADC_INT1					ADC1_1_Handler

#define ADC_INT0_NR					ADC1_0_IRQn			// Overrun/Window
#define ADC_INT1_NR					ADC1_1_IRQn			// Result ready
#define ADC_INT_PRIO				configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2

#define ADC_REG						ADC1

#define ADC2_CLK_APBXMASK			((Mclk *)MCLK)->APBDMASK.bit.ADC0_
#define ADC2_CLK_GCLK_ID			40 //(41 = ADC1)			// Check PCHCTRL in datasheet for index of peripheral
#define ADC2_INT0					ADC0_0_Handler
#define ADC2_INT1					ADC0_1_Handler

#define ADC2_INT0_NR				ADC0_0_IRQn			// Overrun/Window
#define ADC2_INT1_NR				ADC0_1_IRQn			// Result ready
#define ADC2_INT_PRIO				configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2

#define ADC2_REG					ADC0


#ifndef _NVM_SW_CALIB_AREA_BASE
#define _NVM_SW_CALIB_AREA_BASE 0x00800080
#endif

#ifndef ADC_CLK_PCHCTRL
#define ADC_CLK_PCHCTRL				GCLK_PCHCTRL_GEN_GCLK1_Val
#endif

#define ANALOG_TIMEROS_TICKS		125
#define ANALOG_SECOND_PRS			(1000/ANALOG_TIMEROS_TICKS)				// Second prescaler in timer

#define ADC_CHANNELS_MAX			7				// 7 values measured
#define ADC2_CHANNELS_MAX			2				// 2 values measured

volatile bool analog_vcc_calibrate = false;

// Globals
		volatile uint32_t analog_avg_supply_voltage, analog_avg_out_current, analog_avg_out_current20, analog_avg_out_current_max, analog_avg_output_voltage;
		volatile uint32_t analog_avg_out_current_offset = 0, analog_avg_out_current20_offset;
		int16_t analog_avg_temperature, analog_avg_temperature_max;
		uint16_t analog_avg_temperature_logged;
		
static volatile bool adc_calc_busy = false;
static volatile uint8_t adc_calibrate = 0, adc2_calibrate = 0;
static volatile uint32_t adc_calibrate_val = 0, adc2_calibrate_val = 0;
static volatile bool adc_calibrate_busy = false;
static volatile uint32_t adc_calibrate_offset = 0, adc2_calibrate_offset;
static volatile uint32_t adc_calibrate_gain = 1000UL;

	   volatile uint16_t adc_channel_val[ADC_CHANNELS_MAX];				// Low pass values
	   volatile uint16_t adc_channel_last[ADC_CHANNELS_MAX];			// Last measured values
	   volatile uint16_t adc_channel_sel = 0;
	   
	   volatile uint16_t adc2_channel_val[ADC2_CHANNELS_MAX];			// Low pass values
	   volatile uint16_t adc2_channel_last[ADC2_CHANNELS_MAX];			// Last measured values
	   volatile uint16_t adc2_channel_sel = 0;
		
		volatile uint8_t  adc_valid_time = 10;								// Wait one second before values valid
		volatile bool     adc_init_ready = false;
		
//static energy_log_t __attribute__ ((section (".energy_ram"))) energy_log;

//static char text[LOG_TEXT_SIZE + 8];

const uint32_t g_Adc_12bitFullRange = 4096U;

TimerHandle_t  xAnalog_Timer = NULL;

//static int conversions = 0;

static volatile int channel_index = 0, res; 

const uint8_t adc_channels[ADC_CHANNELS_MAX] = { ADC_CH1_INPUTCTRL_MUXPOS, ADC_CH2_INPUTCTRL_MUXPOS, ADC_CH3_INPUTCTRL_MUXPOS, ADC_CH4_INPUTCTRL_MUXPOS, ADC_TINT1_INPUTCTRL_MUXPOS, ADC_TINT2_INPUTCTRL_MUXPOS, ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC };
const uint8_t adc_filters[ADC_CHANNELS_MAX] = { 3, 3, 3, 3, 6, 6, 3 };
	
const uint8_t adc2_channels[ADC2_CHANNELS_MAX] = { EXT_NTC_INPUTCTRL_MUXPOS, ADC_VIN_INPUTCTRL_MUXPOS };
const uint8_t adc2_filters[ADC2_CHANNELS_MAX] = { 6, 6 };

// ADC1
#define AD_CHAN_1			0
#define AD_CHAN_2			1
#define AD_CHAN_3			2
#define AD_CHAN_4			3
#define AD_CHAN_TINT1		4
#define AD_CHAN_TINT2		5
#define AD_CHAN_IOVCC		6

// ADC2
#define AD2_CHAN_NTC		0
#define AD2_CHAN_VSUPPLY	1


// Proto
void Analog_125ms(TimerHandle_t xTimer);
void Analog_Apply_Gain_Correction(uint16_t measured);

#ifdef PCB_REV6	
extern const int8_t ntc_table[512];
#else
extern const uint8_t ntc_table[512];
#endif

extern boot_t boot_data;
extern volatile stepdown_control_t stepdown_ctrl_chx[MAX_DIM_CHANNELS];

#define NVM_SW_CALIB_AREA ((uint32_t *)_NVM_SW_CALIB_AREA_BASE)

/**
 *
 */
void Analog_Init()
{
	MCLK_CRITICAL_SECTION_ENTER();
	ADC_CLK_APBXMASK = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC_CLK_GCLK_ID, ADC_CLK_PCHCTRL | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Generic clock generator 1 = 12MHz
	
	ADC_REG->CTRLA.bit.SWRST = ADC_CTRLA_SWRST;
	while ((ADC_REG->SYNCBUSY.reg) & ADC_SYNCBUSY_SWRST) {
	};
	
	ADC_REG->INTENSET.reg = ADC_INTENSET_RESRDY;
	ADC_REG->INPUTCTRL.reg = AD_GND_REF_INPUTCTRL_MUXPOS;
	ADC_REG->CTRLB.reg = 0;
	ADC_REG->REFCTRL.reg = ADC_REFCTRL_REFCOMP;

	SUPC->VREF.reg = SUPC_VREF_SEL_1V0;			// Also used for DAC!!!
		
	ADC_REG->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV8;
	ADC_REG->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(38);		// 38 clock cycles per sample -> 12-bit + 38 = 50 cycles per sample -> (12MHz / 8) = 1.5MHz) / 50 -> 30.0kHz sampling -> avaraging 4 = 7.5kHz / ADC_CHANNELS_MAX = 1.07kHz per channel  (measured 7.18Hz => 1.03Hz)
	ADC_REG->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 |  ADC_AVGCTRL_ADJRES(2);
	
	// ---------- ADC1 calibration ----------
	uint32_t calval1 = NVM_SW_CALIB_AREA[2];
	ADC_REG->CALIB.reg =
		((calval1 >> 0) & 0x7) |          // BIASCOMP
		(((calval1 >> 3) & 0x7) << 8) |   // BIASR2R
		(((calval1 >> 6) & 0x7) << 12);   // BIASREFBUF		// Search for 'NVM Software Calibration Area Mapping' in datasheet
	
	ADC_REG->CTRLA.bit.ENABLE = true;		// INTREF (internal bandgap reference), Reference buffer offset compensation is enabled
	
	NVIC_DisableIRQ(ADC_INT1_NR);
	NVIC_ClearPendingIRQ(ADC_INT1_NR);
	NVIC_SetPriority(ADC_INT1_NR, ADC_INT_PRIO);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(ADC_INT1_NR);
	
	// GND offset ref
	gpio_set_pin_direction(AD_GND_REF, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(AD_GND_REF, AD_GND_REF_AD);
	
	// Temp measure
	gpio_set_pin_direction(ADC_TINT1, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_TINT1, ADC_TINT1_AD);
	gpio_set_pin_direction(ADC_TINT2, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_TINT2, ADC_TINT2_AD);

	// Input voltage
	gpio_set_pin_direction(ADC_VIN, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_VIN, ADC_VIN_AD);

	// Channels
	gpio_set_pin_direction(ADC_CH1, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_CH1, ADC_CH1_AD);
	gpio_set_pin_direction(ADC_CH2, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_CH2, ADC_CH2_AD);
	gpio_set_pin_direction(ADC_CH3, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_CH3, ADC_CH3_AD);
	gpio_set_pin_direction(ADC_CH4, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(ADC_CH4, ADC_CH4_AD);

	vTaskDelay(2);
	
	adc_calibrate_val = 0;
	adc_calibrate = 16;
	adc_calibrate_busy = true;
	
	ADC_REG->SWTRIG.bit.START = true;
	
	while(adc_calibrate > 0) vTaskDelay(2);
	
	adc_calibrate_offset = adc_calibrate_val / 16;
	adc_calibrate_val = 0;

	adc_calibrate_busy = false;
	
	// ADC2
	MCLK_CRITICAL_SECTION_ENTER();
	ADC2_CLK_APBXMASK = 1;
	MCLK_CRITICAL_SECTION_LEAVE();
	
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC2_CLK_GCLK_ID, ADC_CLK_PCHCTRL | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Set to Generic clock generator 1 = 12MHz
	
	ADC2_REG->CTRLA.bit.SWRST = ADC_CTRLA_SWRST;
	while ((ADC2_REG->SYNCBUSY.reg) & ADC_SYNCBUSY_SWRST) {
	};
	
	ADC2_REG->INTENSET.reg = ADC_INTENSET_RESRDY;
	ADC2_REG->INPUTCTRL.reg = AD2_GND_REF_INPUTCTRL_MUXPOS;
	ADC2_REG->CTRLB.reg = 0;
	ADC2_REG->REFCTRL.reg = ADC_REFCTRL_REFCOMP;
	
	ADC2_REG->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV64;
	ADC2_REG->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(38);		// 38 clock cycles per sample -> 12-bit + 38 = 50 cycles per sample -> (12MHz / 64) = 187.5kHz) / 50 -> 3.75kHz sampling -> avaraging 4 = 0.938kHz / ADC2_CHANNELS_MAX = 0.938kHz per channel (probably a bit lower)
	ADC2_REG->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 |  ADC_AVGCTRL_ADJRES(2);
	
	// ---------- ADC0 calibration ----------
	uint32_t calval0 = NVM_SW_CALIB_AREA[1];
	ADC2_REG->CALIB.reg =
		((calval0 >> 0) & 0x7) |          // BIASCOMP
		(((calval0 >> 3) & 0x7) << 8) |   // BIASR2R
		(((calval0 >> 6) & 0x7) << 12);   // BIASREFBUF		// Search for 'NVM Software Calibration Area Mapping' in datasheet		// Search for 'NVM Software Calibration Area Mapping' in datasheet
	
	ADC2_REG->CTRLA.bit.ENABLE = true;		// INTREF (internal bandgap reference), Reference buffer offset compensation is enabled
	
	NVIC_DisableIRQ(ADC2_INT1_NR);
	NVIC_ClearPendingIRQ(ADC2_INT1_NR);
	NVIC_SetPriority(ADC2_INT1_NR, ADC2_INT_PRIO);					// Prio 0 is highest, 7 is lowest
	NVIC_EnableIRQ(ADC2_INT1_NR);
	
	// GND offset ref
	gpio_set_pin_direction(AD2_GND_REF, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(AD2_GND_REF, AD2_GND_REF_AD);	
	
	// Ext NTC measure
	gpio_set_pin_direction(EXT_NTC, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(EXT_NTC, EXT_NTC_AD);
	
	vTaskDelay(2);
	
	adc2_calibrate_val = 0;
	adc2_calibrate = 16;
	adc_calibrate_busy = true;
	
	ADC2_REG->SWTRIG.bit.START = true;
	
	while(adc2_calibrate > 0) vTaskDelay(2);
	
	adc2_calibrate_offset = adc2_calibrate_val / 16;
	adc2_calibrate_val = 0;

	adc_calibrate_busy = false;
	
	// Start regular
	
	ADC_REG->SWTRIG.bit.START = true;			// Start regular
	ADC2_REG->SWTRIG.bit.START = true;
	
	if(xAnalog_Timer == NULL)
		xAnalog_Timer = xTimerCreate("Analog_Timer", ANALOG_TIMEROS_TICKS, pdTRUE, ( void * ) 0, Analog_125ms);

	if(xAnalog_Timer == NULL)
	{
		GEN_TRACE_INFO("Timer not created analog");				// Todo: what to do
	}
	else if( xTimerStart(xAnalog_Timer, 0) != pdPASS )
	{
		GEN_TRACE_INFO("Timer not started analog");				// Todo: what to do
	}
	adc_init_ready = true;
}

bool Analog_Ready()
{
	return adc_valid_time == 0;
}

void ADC_INT1(void)
{
	if(!adc_calibrate_busy)
	{
		uint16_t oldVal = adc_channel_val[adc_channel_sel];
		uint16_t newVal = ADC_REG->RESULT.reg;
		if(newVal >= adc_calibrate_offset)
			newVal -= adc_calibrate_offset;
		else
			newVal = 0;
		
		uint16_t calcVal;
		if(newVal != oldVal)
		{
			if(oldVal < newVal)
			{
				calcVal = (uint16_t)(oldVal + ((newVal - oldVal) >> adc_filters[adc_channel_sel]));
				if(oldVal == calcVal)
					calcVal++;
			}
			else
			{
				calcVal = (uint16_t)(oldVal - ((oldVal - newVal) >> adc_filters[adc_channel_sel]));
				if(oldVal == calcVal)
					calcVal--;
			}
			adc_channel_val[adc_channel_sel] = calcVal;
			adc_channel_last[adc_channel_sel] = newVal;
		}
		if(adc_channel_sel == AD_CHAN_IOVCC && analog_vcc_calibrate)
			Analog_Apply_Gain_Correction(adc_channel_val[AD_CHAN_IOVCC]);
			
		if(++adc_channel_sel >= ADC_CHANNELS_MAX)
			adc_channel_sel = 0;
		
		ADC_REG->INPUTCTRL.reg = adc_channels[adc_channel_sel];
		ADC_REG->SWTRIG.bit.START = true;
	}
	else if(adc_calibrate > 0)
	{
		adc_calibrate_val += ADC_REG->RESULT.reg;
		if(--adc_calibrate > 0)
			ADC_REG->SWTRIG.bit.START = true;
	}
	
	ADC_REG->INTFLAG.reg = ADC_INTFLAG_RESRDY;
}

void ADC2_INT1(void)
{
	if(!adc_calibrate_busy)
	{
		uint16_t oldVal = adc2_channel_val[adc2_channel_sel];
		uint16_t newVal = ADC2_REG->RESULT.reg;
		if(newVal >= adc2_calibrate_offset)
			newVal -= adc2_calibrate_offset;
		else
			newVal = 0;
		
		uint16_t calcVal;
		if(newVal != oldVal)
		{
			if(oldVal < newVal)
			{
				calcVal = (uint16_t)(oldVal + ((newVal - oldVal) >> adc2_filters[adc2_channel_sel]));
				if(oldVal == calcVal)
					calcVal++;
			}
			else
			{
				calcVal = (uint16_t)(oldVal - ((oldVal - newVal) >> adc2_filters[adc2_channel_sel]));
				if(oldVal == calcVal)
					calcVal--;
			}
			adc2_channel_val[adc2_channel_sel] = calcVal;
			adc2_channel_last[adc2_channel_sel] = newVal;
		}
		if(++adc2_channel_sel >= ADC2_CHANNELS_MAX)
			adc2_channel_sel = 0;
		
		ADC2_REG->INPUTCTRL.reg = adc2_channels[adc2_channel_sel];
		ADC2_REG->SWTRIG.bit.START = true;
	}
	else if(adc2_calibrate > 0)
	{
		adc2_calibrate_val += ADC2_REG->RESULT.reg;
		if(--adc2_calibrate > 0)
			ADC2_REG->SWTRIG.bit.START = true;
	}
	
	ADC2_REG->INTFLAG.reg = ADC_INTFLAG_RESRDY;
}

void Analog_Apply_Gain_Correction(uint16_t measured)
{
	if (measured == 0) return; // avoid division by zero

	// Gain = reference / measured
	// GAINCORR = gain * 0x2000 = (ADC_IO_VCC_TICKS * 0x800) / measured
	uint32_t gaincorr = ((uint32_t)ADC_IO_VCC_TICKS * 0x800U + (measured / 2U)) / measured; // +0.5 rounding

	// Clip to valid 12-bit signed range (0x0000–0x3FFF)
	if (gaincorr > 0x0FFFU) gaincorr = 0x0FFFU;

	//stepdown_ctrl_chx[0].test = gaincorr;
	
	// Apply correction
	ADC_REG->GAINCORR.reg = (uint16_t)gaincorr;
	ADC_REG->OFFSETCORR.reg = 0;      // Optional: only if you also want offset correction
	ADC_REG->CTRLB.bit.CORREN = 1;    // Enable correction

	//while (ADC_REG->SYNCBUSY.bit.CTRLB); // Wait for sync if required
}

void Analog_125ms(TimerHandle_t xTimer)
{
	//stepdown_ctrl_chx[0].test = adc_calibrate_offset;
	if(xTaskGetTickCount() > 1000)			// |Wait for everything to stabilize after power-up before max values are stored
	{
		analog_vcc_calibrate = true;
		
		uint8_t temp = Adc_GetNtc();
		if(appLogData->ntc_temp_max < temp || appLogData->ntc_temp_max == 255)
		{
			appLogData->ntc_temp_max = temp;
			appLogData->change_count++;
		}
		
		uint8_t tint1 = Analog_GetTint1();
		uint8_t tint2 = Analog_GetTint2();
		temp = tint1 > tint2 ? tint1 : tint2;
		if(appLogData->intern_temp_max < temp || appLogData->intern_temp_max == 255)
		{
			appLogData->intern_temp_max = temp;	
			appLogData->change_count++;
		}
		
		uint16_t supply = Analog_GetSupplyVoltage();
		if(appLogData->supply_min > supply || appLogData->supply_min == 255)
		{
			appLogData->supply_min = supply;
			appLogData->change_count++;
		}
		
		if(tint1 == 255 || tint2 == 255)			// Error?
		{
			State_SetError(STATE_ERROR_NTC);
		}
		else
			State_ClearError(STATE_ERROR_NTC);
	}
	if(adc_init_ready)
	{
		if(adc_valid_time > 0)
			adc_valid_time--;
	}
}

/**
 * Get supply voltage in mV
 */
uint32_t Analog_GetSupplyVoltage()
{
	return (Analog_GetSupplyVoltageRaw() * ADC_VIN_MAX)/4095;
}

uint32_t Analog_GetSupplyVoltageRaw()
{
	return (adc2_channel_val[AD2_CHAN_VSUPPLY]);
}


uint8_t Adc_GetTemperature()
{
	uint8_t tint1 = Analog_GetTint1();
	uint8_t tint2 = Analog_GetTint2();
	return tint1 > tint2 ? tint1 : tint2;
}

uint32_t Adc_GetTemperatureRaw()
{
	if( adc_channel_val[AD_CHAN_TINT1] < adc_channel_val[AD_CHAN_TINT2])
		return adc_channel_val[AD_CHAN_TINT1];
	return adc_channel_val[AD_CHAN_TINT2];
}

uint8_t Adc_GetTemperatureMax()
{
	return appLogData->intern_temp_max;
}

uint8_t Analog_GetTint1()
{
	return ntc_table[((adc_channel_val[AD_CHAN_TINT1]) >> 3) & 0x01FF];
}

uint8_t Analog_GetTint2()
{
	return ntc_table[((adc_channel_val[AD_CHAN_TINT2]) >> 3) & 0x01FF];
}

uint32_t Analog_GetTint1Raw()
{
	return adc_channel_val[AD_CHAN_TINT1];
}

uint32_t Analog_GetTint2Raw()
{
	return adc_channel_val[AD_CHAN_TINT2];
}

uint16_t Adc_GetNtc()
{
	return  ntc_table[((adc2_channel_val[AD2_CHAN_NTC]) >> 3) & 0x01FF];	//AT30TSE_DEGREES(ntc);
}

uint8_t Adc_GetNtcMax()
{
	return appLogData->ntc_temp_max;
}

// Ntc per 0.125 (most resolution)
uint16_t Adc_GetNtcRaw()
{
	return adc2_channel_val[AD2_CHAN_NTC];
}

uint16_t Adc_GetVled1Raw()
{
	return (adc_channel_val[AD_CHAN_1]);
}

uint16_t Adc_GetVled1()
{
	return (uint16_t)((Adc_GetVled1Raw() * ADC_VIN_MAX)/4095UL);
}

uint16_t Adc_GetVled1LastRaw()
{
	return (adc_channel_last[AD_CHAN_1]);
}

uint16_t Adc_GetVled2Raw()
{
	return (adc_channel_val[AD_CHAN_2]);
}

uint16_t Adc_GetVled2()
{
	return (uint16_t)((Adc_GetVled2Raw() * ADC_VIN_MAX)/4095UL);
}

uint16_t Adc_GetVled2LastRaw()
{
	return (adc_channel_last[AD_CHAN_2]);
}

uint16_t Adc_GetVled3Raw()
{
	return (adc_channel_val[AD_CHAN_3]);
}

uint16_t Adc_GetVled3()
{
	return (uint16_t)((Adc_GetVled3Raw() * ADC_VIN_MAX)/4095UL);
}

uint16_t Adc_GetVled3LastRaw()
{
	return (adc_channel_last[AD_CHAN_3]);
}

uint16_t Adc_GetVled4Raw()
{
	return (adc_channel_val[AD_CHAN_4]);
}

uint16_t Adc_GetVled4()
{
	return (uint16_t)((Adc_GetVled4Raw() * ADC_VIN_MAX)/4095UL);
}

uint16_t Adc_GetVled4LastRaw()
{
	return (adc_channel_last[AD_CHAN_4]);
}


/**
 * Get supply voltage in mV
 */
uint16_t Analog_GetProcessorTemperature()
{
	return analog_avg_temperature;
}

const uint8_t ntc_table[512] =
{
	255,	255,	255,	255,	255,	255,	195,	186,	178,	171,	166,	161,	156,	152,	148,	145,
	142,	139,	136,	133,	131,	129,	127,	125,	123,	121,	119,	117,	116,	114,	113,	112,
	110,	109,	108,	107,	105,	104,	103,	102,	101,	100,	99,	98,	97,	96,	96,	95,
	94,	93,	92,	92,	91,	90,	89,	89,	88,	87,	87,	86,	85,	85,	84,	84,
	83,	82,	82,	81,	81,	80,	80,	79,	79,	78,	78,	77,	77,	76,	76,	75,
	75,	74,	74,	74,	73,	73,	72,	72,	72,	71,	71,	70,	70,	70,	69,	69,
	69,	68,	68,	67,	67,	67,	66,	66,	66,	65,	65,	65,	64,	64,	64,	64,
	63,	63,	63,	62,	62,	62,	61,	61,	61,	61,	60,	60,	60,	60,	59,	59,
	59,	58,	58,	58,	58,	57,	57,	57,	57,	56,	56,	56,	56,	56,	55,	55,
	55,	55,	54,	54,	54,	54,	53,	53,	53,	53,	53,	52,	52,	52,	52,	52,
	51,	51,	51,	51,	51,	50,	50,	50,	50,	50,	49,	49,	49,	49,	49,	48,
	48,	48,	48,	48,	48,	47,	47,	47,	47,	47,	47,	46,	46,	46,	46,	46,
	45,	45,	45,	45,	45,	45,	45,	44,	44,	44,	44,	44,	44,	43,	43,	43,
	43,	43,	43,	42,	42,	42,	42,	42,	42,	42,	41,	41,	41,	41,	41,	41,
	41,	40,	40,	40,	40,	40,	40,	40,	39,	39,	39,	39,	39,	39,	39,	39,
	38,	38,	38,	38,	38,	38,	38,	37,	37,	37,	37,	37,	37,	37,	37,	36,
	36,	36,	36,	36,	36,	36,	36,	35,	35,	35,	35,	35,	35,	35,	35,	35,
	34,	34,	34,	34,	34,	34,	34,	34,	34,	33,	33,	33,	33,	33,	33,	33,
	33,	33,	32,	32,	32,	32,	32,	32,	32,	32,	32,	31,	31,	31,	31,	31,
	31,	31,	31,	31,	31,	30,	30,	30,	30,	30,	30,	30,	30,	30,	30,	29,
	29,	29,	29,	29,	29,	29,	29,	29,	29,	28,	28,	28,	28,	28,	28,	28,
	28,	28,	28,	27,	27,	27,	27,	27,	27,	27,	27,	27,	27,	27,	26,	26,
	26,	26,	26,	26,	26,	26,	26,	26,	26,	25,	25,	25,	25,	25,	25,	25,
	25,	25,	25,	25,	25,	24,	24,	24,	24,	24,	24,	24,	24,	24,	24,	24,
	24,	23,	23,	23,	23,	23,	23,	23,	23,	23,	23,	23,	23,	22,	22,	22,
	22,	22,	22,	22,	22,	22,	22,	22,	22,	22,	21,	21,	21,	21,	21,	21,
	21,	21,	21,	21,	21,	21,	21,	20,	20,	20,	20,	20,	20,	20,	20,	20,
	20,	20,	20,	20,	19,	19,	19,	19,	19,	19,	19,	19,	19,	19,	19,	19,
	19,	18,	18,	18,	18,	18,	18,	18,	18,	18,	18,	18,	18,	18,	18,	17,
	17,	17,	17,	17,	17,	17,	17,	17,	17,	17,	17,	17,	17,	17,	16,	16,
	16,	16,	16,	16,	16,	16,	16,	16,	16,	16,	16,	16,	16,	15,	15,	15,
	15,	15,	15,	15,	15,	15,	15,	15,	15,	15,	15,	15,	14,	14,	0,	0

};
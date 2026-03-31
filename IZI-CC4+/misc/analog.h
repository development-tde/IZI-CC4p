/*
 * analog.h
 *
 *  Created on: 9 dec. 2020
 *      Author: Milo
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#define ADC_VIN_MAX				54348UL				// Max to be measured in mV (ref 1.0V, 18.4mV/V)
#define ADC_IO_VCC_TICKS		3379				// 4096 = 1V -> 3379, used as gain

#define ADC_RAW_MV(r)			((uint16_t)((r * ADC_VIN_MAX)/4095UL))
#define ADC_RAW_MV32(r)			((uint32_t)((r * ADC_VIN_MAX)/4095UL))
#define ADC_MV_RAW(m)			((uint16_t)((m * 4095UL)/ADC_VIN_MAX))

void Analog_Init();
bool Analog_Ready();
void Analog_Handler();
uint32_t Analog_GetSupplyVoltage();
uint32_t Analog_GetSupplyVoltageRaw();
uint32_t Analog_GetOutputVoltage();
uint16_t Analog_GetProcessorTemperature();
uint32_t Analog_GetTint1Raw();
uint32_t Analog_GetTint2Raw();
uint8_t Analog_GetTint1();
uint8_t Analog_GetTint2();
uint8_t Adc_GetTemperature();
uint8_t Adc_GetTemperatureMax();
uint32_t Adc_GetTemperatureRaw();
uint16_t Adc_GetVled1Raw();
uint16_t Adc_GetVled1();
uint16_t Adc_GetVled2Raw();
uint16_t Adc_GetVled2();
uint16_t Adc_GetVled3Raw();
uint16_t Adc_GetVled3();
uint16_t Adc_GetVled4Raw();
uint16_t Adc_GetVled4();
uint16_t Adc_GetVled1LastRaw();
uint16_t Adc_GetVled2LastRaw();
uint16_t Adc_GetVled3LastRaw();
uint16_t Adc_GetVled4LastRaw();
uint16_t Adc_GetNtc();
uint16_t Adc_GetNtcRaw();
uint16_t Adc_GetNtcUpdate();
uint8_t Adc_GetNtcMax();

#endif /* ANALOG_H_ */

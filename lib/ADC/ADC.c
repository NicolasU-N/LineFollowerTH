/*
 * ADC.c
 * library for atmega 2560 adc configuration
 * 
 * Created: 08/04/2021 06:43:35 p.m.
 * Author: Nicolas
 */
#include "ADC.h"

void ADCInit()
{
	// Select Vref=AVcc
	ADMUX |= (1 << REFS0);
	//set prescaller to 128 and enable ADC // Frequency divisor = 128 -> 16000/128 = 125 KHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADCGetData(uint8_t canal)
{
	// Seleccion del canal del ADC
	ADMUX &= ~0x0F;
	ADMUX |= canal;

	// Encendemos en ADC
	ADCSRA |= (1 << ADEN);
	_delay_us(2); // Esperamos a encender

	// single conversion mode
	ADCSRA |= (1 << ADSC);

	// Esperamos a que muestree, leyendo el flag
	while (!(ADCSRA & (1 << ADIF)))
		;
	ADCSRA |= (1 << ADIF); // Reiniciamos el flag

	// Apagamos el ADC
	ADCSRA &= ~(1 << ADEN);

	return ADC;
}

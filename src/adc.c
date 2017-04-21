// ADC peripheral configuration 

//Developer : Gustavo Nishihara

#include "stm32f407xx.h"
#include "adc.h"

#define SQR1_RESET ((uint32_t)0xFF0FFFFF) 
#define SMPR2_SMP3_RESET ((uint32_t)0xFFFFFFC7) 

void clock_config() //              /to do -> add parameters for adc and gpio selection
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enables ADC1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enables GPIOA which contains ADC1_channel 1
}

void GPIOx_config() //       /to do -> add parameters for which gpio to config and types of configuration
{
	//to select the gpio parameters(input,output,pull-up,analog...) 4 registers must be configured
	GPIOA->MODER |= 0xC; //MODERx[1,0] = 11 for analog in pin x /see GPIO_MODER page 283 manual / to do -> change for mask
	GPIOA->PUPDR &= 0x00; // PUPDRx[1,0] = 00 for analog in pin x /see GPIO_PUPDR page 284 manual / to do -> change for mask
}

void adc_config() //       /to do -> add parameters for types of configuration -> clear the registers before writing stuff
{
	ADC1->CR1 |= 0<<8; // scan mode configuration (single ou multichannel) / to do -> change for mask
	// resolution configuration / to do -> change for mask
	ADC1->CR1 |= 0<<24; 
	ADC1->CR1 |= 0<<25;

	ADC1->CR2 |= 0<<11; // right(1) or left(0) alignment  / to do -> change for mask

	// external trigger enable / to do -> change for mask
	ADC1->CR2 |= 0<<28;
	ADC1->CR2 |= 0<<29;

	ADC1->SQR1 &= SQR1_RESET; // set number of conversions to 1(0000)
	ADC1->CR2 |= ADC_CR2_ADON; // activate ADC

	// sampling time configuration (000) -> 3 cycles see page 425 manual / to do -> change for mask
	ADC1->SMPR2 &= SMPR2_SMP3_RESET;
	ADC1->SMPR2 |= 0<<3; 
	ADC1->SMPR2 |= 0<<2;
	ADC1->SMPR2 |= 0<<1;

	ADC1->SQR3 |= 1<<0; //sets order of conversion  / to do -> change for mask

	ADC1->CR2 |= ADC_CR2_CONT; // continuous conversion mode 
	ADC1->CR2 |= ADC_CR2_SWSTART; // initiates regular group conversion
}

uint16_t adc_value()
{
	return (uint16_t) ADC1->DR;
}


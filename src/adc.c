// ADC peripheral configuration 

//Developer : Gustavo Nishihara

#include "stm32f407xx.h"
#include "adc.h"

#define SQR1_RESET ((uint32_t)0xFF0FFFFF) 
#define SMPR2_SMP3_RESET ((uint32_t)0xFFFFFFC7) 




// TO DO : add all the 16 channels and test them   1:config gpio  2:config sample time  3: set number of conversions 4: set order of conversion

void ADCclock_config() //              /to do -> add parameters for adc and gpio selection
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enables ADC1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enables GPIOA which contains ADC1_channel 1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

void GPIOx_config() //       /to do -> add parameters for which gpio to config and types of configuration
{

	GPIOB->MODER |= GPIO_MODER_MODER0; 
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	GPIOA->MODER |= GPIO_MODER_MODER0; 
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	//ADC1_CH1 -> PA1
	//to select the gpio parameters(input,output,pull-up,analog...) 4 registers must be configured
	GPIOA->MODER |= 0xC; //MODERx[1,0] = 11 for analog in pin x /see GPIO_MODER page 283 manual / to do -> change for mask
	GPIOA->PUPDR &= 0x00; // PUPDRx[1,0] = 00 for analog in pin x /see GPIO_PUPDR page 284 manual / to do -> change for mask
	//ADC1_CH2 -> PA2
	GPIOA->MODER |= GPIO_MODER_MODER2; 
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	//ADC1_CH3 -> PA3
	GPIOA->MODER |= GPIO_MODER_MODER3;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	GPIOA->MODER |= GPIO_MODER_MODER4;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	GPIOA->MODER |= GPIO_MODER_MODER5;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	GPIOA->MODER |= GPIO_MODER_MODER6;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	GPIOA->MODER |= GPIO_MODER_MODER7;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR7);


}

void adc_config_single() //       /to do -> add parameters for types of configuration -> clear the registers before writing stuff
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

void adc_config_multi() //       /to do -> add parameters for types of configuration -> clear the registers before writing stuff
{
	ADC1->CR1 |= 1<<8; // scan mode configuration (single ou multichannel) / to do -> change for mask
	// resolution configuration / to do -> change for mask
	ADC1->CR1 |= 0<<24; 
	ADC1->CR1 |= 0<<25;

	ADC1->CR2 |= 0<<11; // right(1) or left(0) alignment  / to do -> change for mask

	// external trigger enable / to do -> change for mask
	ADC1->CR2 |= 0<<28;
	ADC1->CR2 |= 0<<29;

	ADC1->SQR1 |= ADC_SQR1_L_3;//(ADC_SQR1_L_2|ADC_SQR1_L_1| ADC_SQR1_L_0); // set number of conversions to 8 (ADC_SQR1_L_3| ADC_SQR1_L_0);

	// sampling time configuration (000) -> 3 cycles see page 423 manual 
	ADC1->SMPR2 &= SMPR2_SMP3_RESET;

	//----------Sample Time-----------------
	// 15 cycles CH0
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_0;

	// 15 cycles CH1
	ADC1->SMPR2 |= ADC_SMPR2_SMP1_0;

	// 15 cycles CH2
	ADC1->SMPR2 |= ADC_SMPR2_SMP2_0;
	
	// 15 cycles CH3
	ADC1->SMPR2 |= ADC_SMPR2_SMP3_0;

	// 15 cycles CH4
	ADC1->SMPR2 |= ADC_SMPR2_SMP4_0;

	//15 cycles CH5
	ADC1->SMPR2 |= ADC_SMPR2_SMP5_0;

	//15 cycles CH6
	ADC1->SMPR2 |= ADC_SMPR2_SMP6_0;

	//15 cycles CH7
	ADC1->SMPR2 |= ADC_SMPR2_SMP7_0;

	// 15 cycles CH8
	ADC1->SMPR2 |= ADC_SMPR2_SMP8_0;

	//----------Conversion Order----------------------------

	//CH0 first conversion
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1);

	//CH1 second conversion
	ADC1->SQR3 |= ADC_SQR3_SQ2_0;

	//CH2 third conversion
	ADC1->SQR3 |= ADC_SQR3_SQ3_1;

	//CH3 fourth conversion
	ADC1->SQR3 |= (ADC_SQR3_SQ4_1|ADC_SQR3_SQ4_0);

	// CH4 fifth conversion
	ADC1->SQR3 |= ADC_SQR3_SQ5_2;

	// CH5 sixth conversion
	ADC1->SQR3 |= (ADC_SQR3_SQ6_2|ADC_SQR3_SQ6_0);

	// CH6 seventh conversion
	ADC1->SQR2 |= (ADC_SQR2_SQ7_2|ADC_SQR2_SQ7_1);

	//CH7 eighth conversion
	ADC1->SQR2 |= (ADC_SQR2_SQ8_2|ADC_SQR2_SQ8_1|ADC_SQR2_SQ8_0);

	// CH8 ninth conversion
	ADC1->SQR2 |= ADC_SQR2_SQ9_3;

	//ADC1->SQR2 &= ~(ADC_SQR2_SQ8);
	 //------------------------------------------------------



	//ADC1->CR1 |= ADC_CR1_EOCIE; // enables interrupt for EOC
	ADC1->CR1 |= ADC_CR1_OVRIE; // enables OVR interrupt
	ADC1->CR2 |= ADC_CR2_EOCS; //end of conversion flag
	ADC1->CR2 &= ~(ADC_CR2_CONT); // continuous conversion mode 
	//ADC1->CR2 |= ADC_CR2_SWSTART; // initiates regular group conversion

	ADC1->CR2 &= ~(ADC_CR2_DDS); // activate DMA request while there is data
	//ADC1 -> CR2 |= ADC_CR2_DMA; // enables DMA

	ADC1->CR2 |= ADC_CR2_ADON; // activate ADC
	ADC1->CR2 |= ADC_CR2_DMA; // enables DMA
	//ADC1->CR2 |= ADC_CR2_SWSTART; // initiates regular group conversion
}


uint16_t adc_value()
{
	return (uint16_t) ADC1->DR;
}



#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
#include "DMA.h"
#include "I2C.h"
#include "LSM9DS1.h"
#include "stdio.h"
			
			// TO DO: filter function /calibrate LSM9DS1 /change i2c to interrupt/ use magdwick algorithm/ current source will probably work

#define LSM9DS1_AG_ADDR  0x6B
int8_t adc_counter = 0;
uint16_t result = 0;
char data = 'b';
uint32_t average_result = 0;
volatile int32_t filtered_flex[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int32_t output_data[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int16_t buffer[13];
volatile int32_t flex_data[] = {0x0, 0x0, 0x0, 0x0};
volatile int count = 0;
uint16_t flag = 0;
uint8_t I2C_test = 0;
int16_t accel_data[] = {0x0, 0x0, 0x0};
uint8_t accel_test = 0;
volatile int16_t dummy = 0;
volatile uint8_t systick_flag = 0;




void DMA2_Stream0_IRQHandler()
{
	if (((DMA2 -> LISR)&(DMA_LISR_TCIF0)) == DMA_LISR_TCIF0 )
	{
		systick_flag = 0;
		DMA2 -> LIFCR |= DMA_LIFCR_CTEIF0;
		DMA2 -> LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2 -> LIFCR |= DMA_LISR_HTIF0;
	}
}


void SysTick_Handler()
{
	if (systick_flag == 0)
	{
		ADC1->CR2 &= ~(ADC_CR2_DMA);
		ADC1->CR2 |= ADC_CR2_DMA;
		ADC1->CR2 |= ADC_CR2_SWSTART; 
		systick_flag = 1;
	}
}

void ADC_IRQHandler()
{
	if ((ADC1->SR & ADC_SR_OVR) == ADC_SR_OVR)
	{
		DMA2_Stream0 -> M0AR |= (uint32_t)buffer;
		DMA2_Stream0 -> NDTR |= 0x10;
		ADC1->SR &= ~(ADC_SR_OVR);
		ADC1->CR2 &= ~(ADC_CR2_DMA);
		ADC1->CR2 |= ADC_CR2_DMA;
		ADC1->CR2 |= ADC_CR2_SWSTART;
		systick_flag = 1;
	}
	/*else if ((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC)
	{
		if (adc_counter == 7)
		{
			adc_counter = 0;
			systick_flag = 0;
		}
		buffer[adc_counter] = ADC1->DR;
		adc_counter++;
	}*/
	
}




int main(void)
{
	(void)SysTick_Config(0x334500); //334500 0x19A280
	ClockConfig();
	USARTclock_config();
	I2C_clock_init();
	I2C_gpio_config();
	I2C_config(I2C2);
	LSM9DS1_init();
	GPIO_config();
	USART_config();
	NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_EnableIRQ(ADC_IRQn);
	ADCclock_config();
	GPIOx_config();
	NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA_config();
	DMA2_Stream0 -> M0AR |= (uint32_t)buffer;
	DMA_config2();
	adc_config_multi();
	//DWT->CTRL |= 0x1;


	for(;;)
	{
		//dummy = DWT->CYCCNT;
		I2C_test = I2C_Read(I2C2, LSM9DS1_AG_ADDR, 0x20);
		accel_test = I2C_Read(I2C2, LSM9DS1_AG_ADDR, 0x27);
		if (accel_available())
		{
			count++;
			accel_read(accel_data);
		}
		// change this stuff to a function 
		filtered_flex[0] = filtered_flex[0] + ((buffer[0] - filtered_flex[0])>>4);
		output_data[0] = output_data[0] + ((filtered_flex[0] - output_data[0])>>4);

		filtered_flex[1] = filtered_flex[1] + ((buffer[1] - filtered_flex[1])>>4);
		output_data[1] = output_data[1] + ((filtered_flex[1] - output_data[1])>>4);

		filtered_flex[2] = filtered_flex[2] + ((buffer[2] - filtered_flex[2])>>4);
		output_data[2] = output_data[2] + ((filtered_flex[2] - output_data[2])>>4);

		filtered_flex[3] = filtered_flex[3] + ((buffer[3] - filtered_flex[3])>>4);
		output_data[3] = output_data[3] + ((filtered_flex[3] - output_data[3])>>4);

		filtered_flex[4] = filtered_flex[4] + ((buffer[4] - filtered_flex[4])>>4);
		output_data[4] = output_data[4] + ((filtered_flex[4] - output_data[4])>>4);

		filtered_flex[5] = filtered_flex[5] + ((buffer[5] - filtered_flex[5])>>4);
		output_data[5] = output_data[5] + ((filtered_flex[5] - output_data[5])>>4);

		filtered_flex[6] = filtered_flex[6] + ((buffer[6] - filtered_flex[6])>>4);
		output_data[6] = output_data[6] + ((filtered_flex[6] - output_data[6])>>4);

		filtered_flex[7] = filtered_flex[7] + ((buffer[7] - filtered_flex[7])>>4);
		output_data[7] = output_data[7] + ((filtered_flex[7] - output_data[7])>>4);


		send_data(' ');
		send_data('x');
		send_data(' ');
		print_data((int32_t)accel_data[0]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		print_data((int32_t)accel_data[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		print_data((int32_t)accel_data[2]);
		send_data(' ');
		//send_data('\n');
		//send_data('\r');
		send_data('0');
		send_data(' ');
		print_data(output_data[0]);
		send_data(' ');
		send_data('1');
		send_data(' ');
		print_data(output_data[1]);
		send_data(' ');
		send_data('2');
		send_data(' ');
		print_data(output_data[2]);
		send_data(' ');
		send_data('3');
		send_data(' ');
		print_data(output_data[3]);
		send_data(' ');
		send_data('4');
		send_data(' ');
		print_data(output_data[4]);
		send_data(' ');
		send_data('5');
		send_data(' ');
		print_data(output_data[5]);
		send_data(' ');
		send_data('6');
		send_data(' ');
		print_data(output_data[6]);
		send_data(' ');
		send_data('7');
		send_data(' ');
		print_data(output_data[7]);
		send_data('\n');
		send_data('\r');
		count++;
		//count = DWT->CYCCNT;
		//-----------------------------
	}
}

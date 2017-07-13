
#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
#include "DMA.h"
#include "I2C.h"
#include "LSM9DS1.h"
			

#define LSM9DS1_AG_ADDR  0x6B

uint16_t result = 0;
char data = 'b';
uint32_t average_result = 0;
uint32_t filtered_flex = 0;
volatile uint16_t buffer[3400];
volatile uint32_t flex_data[] = {0x0, 0x0, 0x0, 0x0};
volatile int count = 0;
uint16_t flag = 0;
uint8_t I2C_test = 0;
int16_t accel_data[] = {0x0, 0x0, 0x0};
uint8_t accel_test = 0;
volatile uint32_t maf_data[10];
volatile int dummy = 0;




/*void ADC_IRQHandler()
{
	ADC1->SR &= ~(ADC_SR_EOC);
	count++;
}*/

void DMA2_Stream0_IRQHandler()
{
	//dummy++;
	if ( ( (DMA2 -> LISR)&(DMA_LISR_HTIF0) ) == DMA_LISR_HTIF0)
	{
		//DWT->CYCCNT = 0;
		for (int i = 0; i < 1700; ++i)
		{
			if (i%4 == 0)
			{
				//flex_data[0] += buffer[i];
			}
			else if ((i-1)%4 == 0)
			{
				flex_data[1] += buffer[i];
			}
			else if ((i-2)%4 == 0)
			{
				//flex_data[2] += buffer[i];
			}
			/*else if ((i-3)%4 == 0)
			{
				flex_data[3] += buffer[i];
			}*/
		}

		for (int i = 0; i < 3; ++i)
		{
			flex_data[i] = flex_data[i]/425;
		}
		flag = 1;
		DMA2 -> LIFCR |= DMA_LIFCR_CHTIF0;
	}

	if (((DMA2 -> LISR)&(DMA_LISR_TCIF0)) == DMA_LISR_TCIF0 )
	{
		for (int i = 1700; i < 3400; ++i)
		{
			if (i%4 == 0)
			{
				flex_data[0] += buffer[i];
			}
			else if ((i-1)%4 == 0)
			{
				flex_data[1] += buffer[i];
			}
			else if ((i-2)%4 == 0)
			{
				flex_data[2] += buffer[i];
			}
			/*else if ((i-3)%4 == 0)
			{
				flex_data[3] += buffer[i];
			}*/
		}
		for (int i = 0; i < 3; ++i)
		{
			flex_data[i] = flex_data[i]/425;
		}
		flag = 1;
		//count++;
		DMA2 -> LIFCR |= DMA_LIFCR_CTCIF0;
		//count = DWT->CYCCNT;
	}
}


void SysTick_Handler()
{
	dummy++;
}


// ERROR : just the first conversion is in order after that it gets messy


int main(void)
{
	(void)SysTick_Config(0x19A280);
	ClockConfig();
	USARTclock_config();
	//I2C_clock_init();
	//I2C_gpio_config();
	//I2C_config(I2C2);
	//LSM9DS1_init();
	GPIO_config();
	USART_config();
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
		//I2C_test = I2C_Read(I2C2, LSM9DS1_AG_ADDR, 0x20);
		//accel_test = I2C_Read(I2C2, LSM9DS1_AG_ADDR, 0x27);
		//DWT->CYCCNT = 0;
		//if (accel_available())
		//{
			//count++;
			//accel_read(accel_data);
		//}
		// change this stuff to a function 
		if (flag == 1)
		{
			filtered_flex = filtered_flex*0.0001 + flex_data[1];
			flag = 0;
		}
		//DWT->CYCCNT = 0;
		/*send_data('x');
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
		send_data(' ');*/
		send_data('r');
		send_data(' ');
		print_data((int32_t)filtered_flex);
		send_data('\n');
		send_data('\r');
		count++;
		//count = DWT->CYCCNT;
		//-----------------------------
	}
}

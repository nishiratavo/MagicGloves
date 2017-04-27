
#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
			
uint16_t result = 0;
char data = 'a';
int main(void)
{
	ClockConfig();
	USARTclock_config();
	GPIO_config();
	USART_config();
	ADCclock_config();
	GPIOx_config();
	adc_config();
	for(;;)
	{
		result = adc_value();

		if ((USART3->SR & USART_SR_TXE) ==  USART_SR_TXE)
		{
			send_data(data);
		}
	}
}

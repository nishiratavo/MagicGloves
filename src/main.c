
#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
			
uint16_t result = 0;
char data = 'b';
char converted_data[4];
uint16_t average_result = 0;
uint16_t filtered_result = 0;

void convert_data(uint16_t value)
{
	converted_data[0] = value/1000;
	converted_data[1] = (value/100 - converted_data[0]*10);
	converted_data[2] = (value/10 - (converted_data[0]*100) - (converted_data[1]*10));
	converted_data[3] = (value - (converted_data[0]*1000) - (converted_data[1]*100) - (converted_data[2]*10));
	for (int i = 0; i < 4; ++i)
	{
		converted_data[i] += 48;
	}

}






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
		for (int i = 0; i < 20; ++i)
		{
			result = adc_value();
			average_result += result;
		}
		average_result  = average_result/20;
		filtered_result = filtered_result*0.0001 + average_result;
		convert_data(filtered_result);
		send_data('b');
		send_data(' ');
		send_data(converted_data[0]);
		send_data(converted_data[1]);
		send_data(converted_data[2]);
		send_data(converted_data[3]);
		send_data('\n');
		send_data('\r');
	}
}

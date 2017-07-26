
// USART 3 config

#include "stm32f407xx.h"
#include "UART.h"
#include "stdio.h"



void USARTclock_config()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}

void GPIO_config()
{
	GPIOD->MODER |= 0x20000;   // pin d8 (10) for AF PP 
	GPIOD->OTYPER &= 0x00; 
	GPIOD->PUPDR &= 0x00;
	GPIOD->OSPEEDR |= 0x20000;// see OSPEEDR in datasheet not manual 
	GPIOD->AFR[1] |= 0x7; // AF7(0111) USART123  AFR[x] x = 0 for AFRL and x = 1 for AFRH
}

void USART_config()
{
	USART3->CR2 &= ~USART_CR2_STOP; // set 1 stop bit. bits[13,12]=(00)
	USART3->CR1 &= ~USART_CR1_M; // (0) for 8 data bits
	USART3->CR1 &= ~USART_CR1_PCE; //  (0) for parity control disabled
	USART3->CR1 |= USART_CR1_TE; // trasmitter mode enabled
	USART3->BRR |= 0x7; // fractional part of baud rate
	USART3->BRR |= 0x111<<4; // mantissa of baud rate - baud rate = 9600
	USART3->CR1 |= USART_CR1_UE; // enable USART

}

void send_data(char data)
{
	while(!((USART3->SR & USART_SR_TXE) ==  USART_SR_TXE));
	USART3->DR = data;
}



void convert_data(char *converted_data, volatile int32_t value) // change for general case
{
	if (value < 0)
	{
		converted_data[0] = 45;
		converted_data[1] = - value/10000;
		converted_data[2] =  (- value/1000 - (converted_data[1]*10));
		converted_data[3] =  (- value/100 - (converted_data[1]*100) - (converted_data[2]*10));
		converted_data[4] =  (- value/10 - (converted_data[1]*1000) - (converted_data[2]*100) - (converted_data[3]*10));
		converted_data[5] =  (- value - (converted_data[1]*10000) - (converted_data[2]*1000) - (converted_data[3]*100) - (converted_data[4]*10));
	}
	else
	{
		converted_data[0] = 48;
		converted_data[1] = value/10000;
		converted_data[2] = value/1000 - (converted_data[1]*10);
		converted_data[3] = (value/100 - (converted_data[1]*100) - (converted_data[2]*10));
		converted_data[4] = (value/10 - (converted_data[1]*1000) - (converted_data[2]*100) - (converted_data[3]*10));
		converted_data[5] = (value - (converted_data[1]*10000) - (converted_data[2]*1000) - (converted_data[3]*100) - (converted_data[4]*10));
	}	
	for (int i = 1; i < 6; ++i)
	{
		converted_data[i] += 48;
	}
}

void print_data(volatile int32_t data)
{
	char converted_data[6];
	convert_data(converted_data, data);
	send_data(converted_data[0]);
	send_data(converted_data[1]);
	send_data(converted_data[2]);
	send_data(converted_data[3]);
	send_data(converted_data[4]);
	send_data(converted_data[5]);

}

uint8_t return_float_index(char *data)
{
	uint8_t index = 0;
	for (int i = 0; i < 10; ++i)
	{
		if (data[i] == 46)
		{
			index = i + 4;
			break;
		}
	}
	return index;
}

void print_float(float data)
{
	char converted[10];
	snprintf(converted, sizeof(converted), "%.3f", data);
	for (int i = 0; i < return_float_index(converted); ++i)
	{
		send_data(converted[i]);
	}
}


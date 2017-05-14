
// USART 3 config

#include "stm32f407xx.h"
#include "UART.h"



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
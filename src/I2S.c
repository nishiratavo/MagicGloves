#include "stm32f407xx.h"
#include "I2S.h"


void i2s_init()
{

	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	

}
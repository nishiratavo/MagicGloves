#include "stm32f407xx.h"
#include "I2S.h"


#define RCC_PLLI2SCFGR_PLLI2SN_Pos 6
#define RCC_PLLI2SCFGR_PLLI2SR_Pos 28

void i2s_init()
{
	// spi and gpioc clock init
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	/*
	 * I2S PLL configuration
	 * Source: 2MHz
	 * Multiplier: 86
	 * Divider: 2
	 * Final clock: 86 Mhz
	 */
	RCC->PLLI2SCFGR = (86 << RCC_PLLI2SCFGR_PLLI2SN_Pos) |
		(2 << RCC_PLLI2SCFGR_PLLI2SR_Pos);
	RCC->CR |= RCC_CR_PLLI2SON;

	// Waits until the I2S PLL is stable
	while((RCC->CR & RCC_CR_PLLI2SRDY) == 0);

	// gpio config
	// Alternate function 
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	GPIOC->MODER |= GPIO_MODER_MODER10_1; 
	GPIOC->MODER |= GPIO_MODER_MODER12_1; 
	GPIOA->MODER |= GPIO_MODER_MODER4_1; 
	// Push-pull
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_10;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_12; 
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4; 
	// High-speed
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1; 
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1; 
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_1; 
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1; 
	
	// Alternate function I2S3
	GPIOC->AFR[0] |= 6<<28; // PC7
	GPIOC->AFR[1] |= 6<<8; // PC10
	GPIOC->AFR[1] |= 6<<16; // PC12
	GPIOA->AFR[0] |= 6<<16; // PA4


	// I2S3 config
	// audio sample frequency config
	SPI3->I2SPR |= SPI_I2SPR_ODD;
	SPI3->I2SPR |= 10;
	SPI3->I2SPR |= SPI_I2SPR_MCKOE;

	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SMOD; // I2S mode select
	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1; // Master Transmitter mode
	SPI3->I2SCFGR &= ~SPI_I2SCFGR_I2SSTD; // I2S philips standard
	SPI3->I2SCFGR &= ~SPI_I2SCFGR_CKPOL; // clock steady state low level
	SPI3->I2SCFGR &= ~SPI_I2SCFGR_DATLEN; // 16 bit data length

	// dma init


	SPI3->CR2 |= SPI_CR2_TXDMAEN;; // Enable Transmitter DMA
	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE; // Enable I2S3 peripheral


}

uint8_t get_status(uint32_t SPI_I2S_FLAG)
{
if ((SPI3->SR & SPI_I2S_FLAG) == SPI_I2S_FLAG)
  {
    /* SPI_I2S_FLAG is set */
    return 1;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    return 0;
  }
}
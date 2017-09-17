#include "stm32f407xx.h"
#include "CS43L22.h"
#include "I2C.h"

void I2C1_busy_errata()
{
	I2C1->CR1 &= ~(I2C_CR1_PE);
	GPIOB->MODER &= ~(GPIO_MODER_MODER9); // reset mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER6); // reset mode
	GPIOB->MODER |= GPIO_MODER_MODER9_0; // General Purpose Output SDA
	GPIOB->MODER |= GPIO_MODER_MODER6_0; // General Purpose Output SCL
	GPIOB->OTYPER |= GPIO_OTYPER_OT_9; // open-drain SDA
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6; // open-drain SCL
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR9_0; // pull-up PB7
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // pull-up PB8
	GPIOB->ODR |= GPIO_ODR_ODR_9; // set SDA high
	GPIOB->ODR |= GPIO_ODR_ODR_6; // set  SCL high
	//GPIOB->ODR |= GPIO_ODR_ODR_9; // set SDA high
	while(!( (((GPIOB->IDR) & (GPIO_IDR_IDR_6)) == GPIO_IDR_IDR_6) && (((GPIOB->IDR & GPIO_IDR_IDR_9)== GPIO_IDR_IDR_9)))) // check high
	{
		asm("nop");
	}
	GPIOB->ODR &= ~(GPIO_ODR_ODR_9); // set SDA low
	while(!((GPIOB->IDR & GPIO_IDR_IDR_9) == 0)) // check SDA low
	{
		asm("nop");
	}
	GPIOB->ODR &= ~(GPIO_ODR_ODR_6); // set SCL low
	while(!((GPIOB->IDR & GPIO_IDR_IDR_6) == 0)) // check SCL low
	{
		asm("nop");
	}
	GPIOB->ODR |= GPIO_ODR_ODR_6; // set SCL high
	while(!((GPIOB->IDR & GPIO_IDR_IDR_6) == GPIO_IDR_IDR_6)) // check SCL high
	{
		asm("nop");
	}
	GPIOB->ODR |= GPIO_ODR_ODR_9; // set SDA high
	while(!((GPIOB->IDR & GPIO_IDR_IDR_9) == GPIO_IDR_IDR_9)) // check SDA high
	{
		asm("nop");
	}
	GPIOB->MODER &= ~(GPIO_MODER_MODER9); // reset mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER6); // reset mode
	GPIOB->MODER |= GPIO_MODER_MODER9_1; // alternate function SDA
	GPIOB->MODER |= GPIO_MODER_MODER6_1; // alternate function SCL
	GPIOB->AFR[0] |= 1<<26; // alternate function I2C PB6
	GPIOB->AFR[1] |= 1<<6;  //alternate function I2C PB6
	GPIOB->OTYPER |= GPIO_OTYPER_OT_9; // set SDA open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6; // set SCL open-drain
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR9_0; // pull-up SDA
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // pull-up SCL
	I2C1->CR1 |= I2C_CR1_SWRST; // set SWRST bit
	I2C1->CR1 &= ~(I2C_CR1_SWRST); // clear SWRST bit
}


void cs43l22_init()
{
	//clock init
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	//gpio init
	GPIOB->MODER |= GPIO_MODER_MODER6_1; // alternate function PB9
	GPIOB->MODER |= GPIO_MODER_MODER9_1; // alternate function PB11
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6; // open-drain PB7
	GPIOB->OTYPER |= GPIO_OTYPER_OT_9; // open-drain PB8
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; // high-speed PB7
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; // high-speed PB8
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0; // pull-up PB7
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // pull-up PB8
	//GPIOB->AFR[0] |= 0x44000000; // alternate function I2C PB7
	GPIOB->AFR[0] |= 1<<26; // alternate function I2C PB6
	GPIOB->AFR[1] |= 1<<6; // alternate function I2C PB6

	// keeps cs43l22 off
	GPIOD->MODER &= ~(GPIO_MODER_MODER4); // reset mode
	GPIOD->MODER |= GPIO_MODER_MODER4_0; // General Purpose Output SDA
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT_4; // T
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1; // 50MHz
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR4_1; // pull down
	GPIOD->ODR |= 0<<4; // resets PD4

	// i2c init
	I2C1_busy_errata();
	I2C1->CR2 |= (I2C_CR2_FREQ_5|I2C_CR2_FREQ_3|I2C_CR2_FREQ_1);
	//I2Cx->CR2 |= (I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN); // interrupts enabled
	I2C1->CCR &= ~I2C_CCR_FS;
	I2C1->CCR |= 0xD2; //  CCR*Tclk = 1/42MHz
	I2C1->TRISE &= 0xFFC0;
	I2C1->TRISE |= 0x2B;      // (1000ns/(1/42MHz) = 42) TRISE = 42 + 1
	I2C1->CR1 |= I2C_CR1_PE;

}

void cs43l22_ctrl_config()
{
	uint32_t delaycount = 0;
	uint8_t reg;

	GPIOD->ODR |= 1<<4; // turns on cs43l22
	delaycount = 1000000;
	while (delaycount > 0)
	{
		delaycount--;
	}
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PLAYBACK_CTRL1, 0x01);
	//init sequence
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x00, 0x99);
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x47, 0x80);
	reg = I2C_Read(I2C1, CODEC_I2C_ADDRESS, 0x32);
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x32, reg|0x80);
	reg = I2C_Read(I2C1, CODEC_I2C_ADDRESS, 0x32);
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x32, reg&(~0x80);
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x00, 0x00);
	//end of init sequence

	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PWR_CTRL2, 0xAF); // turns on headphone channels
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PLAYBACK_CTRL1, 0x70); // headphone gain -> 0.6047
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_CLK_CTRL, 0x81); // auto detect clock
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_IF_CTRL1, 0x07); // data size and format
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x0A, 0x00); // soft ramp zero cross disabled
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x27, 0x00); // limits signal in 0 dB
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, 0x1F, 0x0F); // treble and bass gain
	I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PWR_CTRL1, 0x9E); // turns on dac 


}
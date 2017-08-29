
#include "stm32f407xx.h"
#include "I2C.h"

/*#define MASTER_MODE_SELECT  ((uint32_t)0x00030001)
#define MASTER_TRANSMITTER_MODE  ((uint32_t)0x00070082)
#define MASTER_RECEIVER_MODE ((uint32_t)0x00030002)
#define MASTER_BYTE_TRANSMITTED ((uint32_t)0x00070084)
#define MASTER_BYTE_RECEIVED ((uint32_t)0x00030040)*/

volatile uint8_t start_flag = 0;
volatile uint8_t master_mode_select = 0;
volatile uint8_t master_transmitter_mode = 0;
volatile uint8_t master_receiver_mode = 0;
volatile uint8_t master_byte_transmitted = 0;
volatile uint8_t master_byte_received = 0;
volatile uint8_t done_flag = 0;
volatile uint8_t i2c_supplier = 0;
volatile uint8_t i2c_collector = 0;
volatile uint8_t i2c_isr_free = 1;
volatile uint8_t first_time = 1;

i2c_address i2c_buffer[32];

// TO DO : change while content for I2C_check_event function | need to add read and write of more than one byte
// use enum for configurations
// change all functions to I2C_TypeDef

void I2C_clock_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

void I2C_gpio_config() // I2C2_SDA -> PB11 and  I2C2_SCL -> PB10
{
	GPIOB->MODER |= GPIO_MODER_MODER11_1; // alternate function PB11
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // alternate function PB11
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11; // open-drain PB7
	GPIOB->OTYPER |= GPIO_OTYPER_OT_10; // open-drain PB8
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1; // high-speed PB7
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1; // high-speed PB8
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0; // pull-up PB7
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; // pull-up PB8
	//GPIOB->AFR[0] |= 0x44000000; // alternate function I2C PB7
	GPIOB->AFR[1] |= 0x4400; // alternate function I2C PB11 and PB10
}

void I2C_acknowledge(char config) // 'E' for enable 'D' for disable
{
	if (config == 'E')
	{
		I2C2->CR1 |= I2C_CR1_ACK;
	}
	else if (config == 'D')
	{
		I2C2->CR1 &= ~(I2C_CR1_ACK);
	}
}

uint8_t I2C_check_event(uint32_t event)
{
	volatile uint32_t flag = 0;
	volatile uint32_t sr1 = 0;
	volatile uint32_t sr2 = 0;

	sr1 = I2C2->SR1; 
	sr2 = I2C2->SR2;
	sr2 = sr2<<16;

	flag = (sr1|sr2);
	if ((flag & event) == event)
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}

//it seems that the slave device interfere with the SDA line, 
// making the 11th bit in IDR to not go high, leaving the code in a while loop
// need to find a solution
void I2C_busy_errata() // change for all I2Cs.   
{
	I2C2->CR1 &= ~(I2C_CR1_PE);
	GPIOB->MODER &= ~(GPIO_MODER_MODER11); // reset mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER10); // reset mode
	GPIOB->MODER |= GPIO_MODER_MODER11_0; // General Purpose Output SDA
	GPIOB->MODER |= GPIO_MODER_MODER10_0; // General Purpose Output SCL
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11; // open-drain SDA
	GPIOB->OTYPER |= GPIO_OTYPER_OT_10; // open-drain SCL
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0; // pull-up PB7
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; // pull-up PB8
	GPIOB->ODR |= GPIO_ODR_ODR_11; // set SDA high
	GPIOB->ODR |= GPIO_ODR_ODR_10; // set  SCL high
	//GPIOB->ODR |= GPIO_ODR_ODR_11; // set SDA high
	while(!( (((GPIOB->IDR) & (GPIO_IDR_IDR_10)) == GPIO_IDR_IDR_10) && (((GPIOB->IDR & GPIO_IDR_IDR_11)== GPIO_IDR_IDR_11)))) // check high
	{
		asm("nop");
	}
	GPIOB->ODR &= ~(GPIO_ODR_ODR_11); // set SDA low
	while(!((GPIOB->IDR & GPIO_IDR_IDR_11) == 0)) // check SDA low
	{
		asm("nop");
	}
	GPIOB->ODR &= ~(GPIO_ODR_ODR_10); // set SCL low
	while(!((GPIOB->IDR & GPIO_IDR_IDR_10) == 0)) // check SCL low
	{
		asm("nop");
	}
	GPIOB->ODR |= GPIO_ODR_ODR_10; // set SCL high
	while(!((GPIOB->IDR & GPIO_IDR_IDR_10) == GPIO_IDR_IDR_10)) // check SCL high
	{
		asm("nop");
	}
	GPIOB->ODR |= GPIO_ODR_ODR_11; // set SDA high
	while(!((GPIOB->IDR & GPIO_IDR_IDR_11) == GPIO_IDR_IDR_11)) // check SDA high
	{
		asm("nop");
	}
	GPIOB->MODER &= ~(GPIO_MODER_MODER11); // reset mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER10); // reset mode
	GPIOB->MODER |= GPIO_MODER_MODER11_1; // alternate function SDA
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // alternate function SCL
	GPIOB->AFR[1] |= 0x4400; // set SDA and SCL as alternate function
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11; // set SDA open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_10; // set SCL open-drain
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0; // pull-up SDA
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; // pull-up SCL
	I2C2->CR1 |= I2C_CR1_SWRST; // set SWRST bit
	I2C2->CR1 &= ~(I2C_CR1_SWRST); // clear SWRST bit
}


void I2C_config(I2C_TypeDef* I2Cx)
{
	I2C_busy_errata();
	I2Cx->CR2 |= (I2C_CR2_FREQ_5|I2C_CR2_FREQ_3|I2C_CR2_FREQ_1);
	I2Cx->CR2 |= (I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN); // interrupts enabled
	I2Cx->CCR |= I2C_CCR_FS;
	I2Cx->CCR |= 0x32; //  3*CCR*Tscl = 1/42MHz
	I2Cx->TRISE &= 0xFFC0;
	I2Cx->TRISE |= 0xD;      // in Fm mode, the maximum allowed SCL rise time is 300 ns (300ns/(1/42MHz) = 12.6) TRISE = 12 + 1
	I2Cx->CR1 |= I2C_CR1_PE; // enable I2C
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t adress, char direction, char check_busy)
{
	if (check_busy == 'T')
	{
		while((I2Cx->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY); // waits for line to be free
	}
	else if (check_busy == 'F')
	{

	}
	I2Cx->CR1 |= I2C_CR1_START; // send start condition
	while(!I2C_check_event(MASTER_MODE_SELECT));// see if SB, MSL and BUSY flag are set
	if (direction == 'R') // for read LSB is set
	{
		adress |= I2C_OAR1_ADD0;
	}
	else if(direction == 'W') // for write LSB is reset
	{
		adress &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
	}

	I2Cx->DR = adress;

	if (direction == 'W')
	{
		while(!I2C_check_event(MASTER_TRANSMITTER_MODE)); //see if BUSY, MSL, ADDR, TXE and TRA flags are set
	}
	else if (direction == 'R')
	{
		while(!I2C_check_event(MASTER_RECEIVER_MODE)); // see if BUSY, MSL and ADDR flags are set
	}

}

void I2C_write(uint8_t data)
{
	I2C2->DR = data;
	while(!I2C_check_event(MASTER_BYTE_TRANSMITTED)); // see if TRA, BUSY, MSL, TXE and BTF flags are set
}

uint8_t I2C_read_ack()
{
	I2C_acknowledge('E');
	while(!I2C_check_event(MASTER_BYTE_RECEIVED));  // see if BUSY, MSL and RXNE flags are set
	uint8_t data = (uint8_t)I2C2->DR;
	return data;
}

uint8_t I2C_read_nack()
{
	I2C_acknowledge('D');
	I2C_stop();
	while(!I2C_check_event(MASTER_BYTE_RECEIVED)); // see if BUSY, MSL and RXNE flags are set
	uint8_t data = (uint8_t)I2C2->DR;
	return data;
}

void I2C_stop()
{
	I2C2->CR1 |= I2C_CR1_STOP;
}

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD)
{
	I2C_start(I2Cx, SAD<<1,'W','T');
	I2C_write(RAD);
	I2C_start(I2Cx, SAD<<1,'R','F');
	return I2C_read_nack();
}

void I2C_Read_IT(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t data)
{

	if (start_flag == 1) // State 1 -> send start
	{
		while((I2Cx->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY);
		I2Cx->CR1 |= I2C_CR1_START;
		start_flag = 0;
	}
	else if (master_mode_select == 1) // State 2 -> send SAD for writing
	{
		SAD = SAD<<1;
		SAD &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
		I2Cx->DR = SAD;
		master_mode_select = 2;
	}
	else if (master_transmitter_mode == 1) // State 3 -> send RAD
	{
		I2Cx->DR = RAD;
		master_transmitter_mode = 0;
	}
	else if (master_byte_transmitted == 1) // State 4 -> send reStart
	{
		I2Cx->CR1 |= I2C_CR1_START;
		master_byte_transmitted = 0;
	}
	else if (master_mode_select == 3) // State 5 -> send SAD for reading
	{
		SAD |= I2C_OAR1_ADD0;
		master_mode_select = 0;
	}
	else if (master_receiver_mode == 1) // State 6 -> send nack and stop
	{
		I2C_acknowledge('D');
		I2C_stop();
		master_receiver_mode = 0;
	}
	else if (master_byte_received == 1) // State 7 -> read data
	{
		data = (uint8_t)I2C2->DR;
		master_byte_received = 0;
		start_flag = 1;
	}
}

void I2C_Read_Many(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t *data, uint8_t count)
{
	I2C_start(I2Cx, SAD<<1,'W','T');
	I2C_write(RAD);
	I2C_start(I2Cx, SAD<<1,'R','F');
	for (uint8_t i = 0; i < count; ++i)
	{
		if (i == count - 3)
		{
			while(!I2C_check_event(1<<2));
			I2C_acknowledge('D');
			data[i] = (uint8_t)I2Cx->DR;
		}
		else if (i == count - 2)
		{
			while(!I2C_check_event(1<<2));
			I2C_stop();
			data[i] = (uint8_t)I2Cx->DR;
		}
		else if (i == count - 1)
		{
			while(!I2C_check_event(MASTER_BYTE_RECEIVED));
			data[i] = (uint8_t)I2C2->DR;
		}
		else
		{
			data[i] = I2C_read_ack();
		}
	}
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t data)
{
	I2C_start(I2Cx, SAD<<1,'W','T');
	I2C_write(RAD);
	I2C_write(data);
	I2C_stop();
}

void I2C_write_buffer(i2c_address *buffer, i2c_address *data)
{
	if (i2c_supplier == (i2c_collector - 1)) 
	{
		return;
	}
	buffer[i2c_supplier].SAD = data->SAD;
	buffer[i2c_supplier].RAD = data->RAD;
	buffer[i2c_supplier].counter = data->counter;
	buffer[i2c_supplier].pointer_data = data->pointer_data;
	i2c_supplier++;
	i2c_supplier &= 0x1F; 
}

void I2C_read_buffer(i2c_address *buffer, i2c_address *data)
{
	
	if (i2c_collector == i2c_supplier) 
	{
		return;
	}
	data->SAD = buffer[i2c_collector].SAD;
	data->RAD = buffer[i2c_collector].RAD;
	data->counter = buffer[i2c_collector].counter;
	data->pointer_data = buffer[i2c_collector].pointer_data;
	i2c_collector ++;
	i2c_collector &= 0x1F; 
}

void i2c_read_it(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t counter, uint8_t *data)
{
	i2c_address i2c_read;
	i2c_read.SAD = SAD;
	i2c_read.RAD = RAD;
	i2c.read.counter = counter;
	i2c_read.pointer_data = data;
	I2C_write_buffer(i2c_buffer, &i2c_read);
	if (i2c_isr_free == 1)
	{
		i2c_isr_free = 0;
		I2Cx->CR1 |= I2C_CR1_START;
	}
	
}


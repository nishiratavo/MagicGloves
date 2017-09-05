
#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
#include "DMA.h"
#include "I2C.h"
#include "LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "stdio.h"
#include "circular_buffer.h"
#include "clockconfig.h"
//#include "MadgwickAHRS.h"
			

			//one is faster than the other -> depends on order and it shouldn't

			// TO DO: test with other sensors
			// TO DO: filter function / use madgwick algorithm/ set I2S / set DAC/

			// circular buffer for ADC data
			

#define LSM9DS1_AG_ADDR  0x6B
#define LSM9DS1_M_ADDR  0x1E
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_MAGNETOMETER_4   0.00014

int8_t adc_counter = 0;
uint16_t result = 0;
char data = 'b';
uint32_t average_result = 0;
uint8_t i2c_counter = 0;
uint8_t i2c_data_counter = 0;
volatile int32_t filtered_flex[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int32_t output_data[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int16_t buffer[13];
volatile int32_t flex_data[] = {0x0, 0x0, 0x0, 0x0};
volatile int count = 0;
uint8_t I2C_test = 0;
volatile int16_t accel_data[] = {0x0, 0x0, 0x0};
volatile int16_t gyro_data[] = {0x0, 0x0, 0x0};
volatile int16_t mag_data[] = {0x0, 0x0, 0x0};
uint8_t accel_test[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t gyro_test[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t mag_test[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int16_t dummy = 0;
volatile uint8_t systick_flag = 0;
volatile float accel_float[3];
volatile float gyro_float[3];
volatile float mag_float[3];
int32_t aBias[] = {0, 0, 0};
int32_t gBias[] = {0, 0, 0};

volatile uint32_t flag = 0;
volatile uint32_t sr1 = 0;
volatile uint32_t sr2 = 0;


i2c_address i2c_buffer[32];


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

void I2C2_EV_IRQHandler()
{
	uint8_t SAD;
	sr1 = I2C2->SR1; 
	sr2 = I2C2->SR2;
	sr2 = sr2<<16;
	flag = (sr1|sr2);

	/*if (start_flag == 1) // State 1 -> send SAD for writing
	{
		start_flag = 0;
		uint8_t SAD = i2c_buffer[i2c_collector].SAD<<1;
		SAD &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
		I2C2->DR = SAD;
	}*/
	if (((flag & MASTER_MODE_SELECT) == MASTER_MODE_SELECT ) && master_mode_select == 0) // State 2
	{
		SAD = i2c_buffer[i2c_collector].SAD<<1;
		SAD &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
		I2C2->DR = SAD;
		master_mode_select = 1;
	}
	else if ((flag & MASTER_TRANSMITTER_MODE) == MASTER_TRANSMITTER_MODE) // State 3 -> send RAD
	{
		I2C2->DR = i2c_buffer[i2c_collector].RAD;
	}
	else if ((flag & MASTER_BYTE_TRANSMITTED) == MASTER_BYTE_TRANSMITTED) // State 4 -> send reStart
	{
		I2C2->CR1 |= I2C_CR1_START;
	}
	else if (((flag & MASTER_MODE_SELECT) == MASTER_MODE_SELECT ) && master_mode_select == 1) // State 5 -> send SAD for reading
	{
		SAD = i2c_buffer[i2c_collector].SAD<<1;
		SAD |= I2C_OAR1_ADD0;
		I2C2->DR = SAD;
		master_mode_select = 0;
	}
	else if (((flag & MASTER_RECEIVER_MODE) == MASTER_RECEIVER_MODE) && (i2c_buffer[i2c_collector].counter > 0)) // 
	{
		i2c_counter = i2c_buffer[i2c_collector].counter;
		I2C2->CR1 |= I2C_CR1_ACK;

	}
	/*else if ((flag & I2C_SR1_BTF) == I2C_SR1_BTF) // 
	{
		I2C2->CR1 |= I2C_CR1_ACK;

	}*/
	else if ((flag & MASTER_RECEIVER_MODE) == MASTER_RECEIVER_MODE) // State 6 -> send nack and stop
	{
		I2C2->CR1 &= ~(I2C_CR1_ACK);
		I2C2->CR1 |= I2C_CR1_STOP;
	}
	else if ((flag & MASTER_BYTE_RECEIVED) == MASTER_BYTE_RECEIVED) // State 7 -> read data
	{
		if (i2c_counter == 2)
		{
			I2C2->CR1 &= ~(I2C_CR1_ACK);
		}
		if (i2c_counter == 1)
		{
			I2C2->CR1 |= I2C_CR1_STOP;
		}

		i2c_buffer[i2c_collector].pointer_data[i2c_data_counter] = (uint8_t)I2C2->DR;

		if (i2c_counter > 3)
		{
			I2C2->CR1 |= I2C_CR1_ACK;
		}

		i2c_data_counter++;
		i2c_counter--;

		if (i2c_counter == 0)
		{
			i2c_collector++;
			i2c_collector &= 0x1F;
			i2c_data_counter = 0;
			i2c_counter = 0;
			i2c_isr_free = 1;
		}
		//i2c_data_counter++;
		//i2c_counter--;


		/*i2c_buffer[i2c_collector].pointer_data[1] = (uint8_t)I2C2->DR;
		i2c_collector++;
		i2c_collector &= 0x1F;
		i2c_isr_free = 1;*/
	}


}




int main(void)
{
	ClockConfig();
	USARTclock_config();
	I2C_clock_init();
	I2C_gpio_config();
	I2C_config(I2C2);
	NVIC_SetPriority(I2C2_EV_IRQn, 2);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	LSM9DS1_init();
	//accel_gyro_calibrate(aBias, gBias);
	GPIO_config();
	USART_config();
	/*NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_EnableIRQ(ADC_IRQn);
	ADCclock_config();
	GPIOx_config();
	NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA_config();
	DMA2_Stream0 -> M0AR |= (uint32_t)buffer;
	DMA_config2();
	adc_config_multi();
	(void)SysTick_Config(0x334500); //334500 0x19A280*/
	//DWT->CTRL |= 0x1;
	I2C2->CR2 |= (I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN);


	for(;;)
	{
		//dummy = DWT->CYCCNT;
		//I2C_test = I2C_Read(I2C2, LSM9DS1_AG_ADDR, 0x20);
		//I2C_Read_IT(I2C2, LSM9DS1_AG_ADDR, 0x0F, accel_test);
		//i2c_read_it(I2C2,LSM9DS1_AG_ADDR, OUT_X_L_XL|0x80, 6, accel_test);
		i2c_read_it(I2C2,LSM9DS1_AG_ADDR, OUT_X_L_G|0x80, 6, gyro_test);
		gyro_data[0] = (gyro_test[1]<<8) | gyro_test[0];
		gyro_data[1] = (gyro_test[3]<<8) | gyro_test[2];
		gyro_data[2] = (gyro_test[5]<<8) | gyro_test[4];

		gyro_float[0] = SENSITIVITY_GYROSCOPE_245*gyro_data[0];
		gyro_float[1] = SENSITIVITY_GYROSCOPE_245*gyro_data[1];
		gyro_float[2] = SENSITIVITY_GYROSCOPE_245*gyro_data[2];

		i2c_read_it(I2C2,LSM9DS1_AG_ADDR, OUT_X_L_XL|0x80, 6, accel_test);
		accel_data[0] = (accel_test[1]<<8) | accel_test[0];
		accel_data[1] = (accel_test[3]<<8) | accel_test[2];
		accel_data[2] = (accel_test[5]<<8) | accel_test[4];

		accel_float[0] = SENSITIVITY_ACCELEROMETER_2*accel_data[0];
		accel_float[1] = SENSITIVITY_ACCELEROMETER_2*accel_data[1];
		accel_float[2] = SENSITIVITY_ACCELEROMETER_2*accel_data[2];

		i2c_read_it(I2C2,LSM9DS1_M_ADDR, OUT_X_L_M|0x80, 6, mag_test);
		mag_data[0] = (mag_test[1]<<8) | mag_test[0];
		mag_data[1] = (mag_test[3]<<8) | mag_test[2];
		mag_data[2] = (mag_test[5]<<8) | mag_test[4];

		mag_float[0] = SENSITIVITY_MAGNETOMETER_4*mag_data[0];
		mag_float[1] = SENSITIVITY_MAGNETOMETER_4*mag_data[1];
		mag_float[2] = SENSITIVITY_MAGNETOMETER_4*mag_data[2];

		//i2c_read_it(I2C2,LSM9DS1_M_ADDR, 0x0F, 0, &I2C_test);
		/*if (accel_available())
		{
			count++;
			accel_g(accel_float, aBias);
			//accel_read(accel_data, aBias);
		}
		if (gyro_available())
		{
			gyro_dps(gyro_float, gBias);
		}*/
		//MadgwickAHRSupdateIMU(gyro_float[0], gyro_float[1], gyro_float[2], accel_float[0], accel_float[1], accel_float[2]);
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


		send_data('x');
		send_data(' ');
		print_float(accel_float[0]);
		//print_data((int32_t)accel_data[0]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		print_float(accel_float[1]);
		//print_data((int32_t)accel_data[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		print_float(accel_float[2]);
		//print_data((int32_t)accel_data[2]);
		send_data(' ');
		send_data('x');
		send_data(' ');
		print_float(gyro_float[0]);
		//print_data((int32_t)gyro_data[0]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		print_float(gyro_float[1]);
		//print_data((int32_t)gyro_data[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		print_float(gyro_float[2]);
		//print_data((int32_t)gyro_data[2]);

		send_data(' ');
		send_data('x');
		send_data(' ');
		//print_float(gyro_float[0]);
		print_float(mag_float[0]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		//print_float(gyro_float[1]);
		print_float(mag_float[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		//print_float(gyro_float[2]);
		print_float(mag_float[2]);
		/*send_data(' ');
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
		print_data(output_data[7]);*/
		send_data('\n');
		send_data('\r');
		count++;
		//count = DWT->CYCCNT;
		//-----------------------------
	}
}

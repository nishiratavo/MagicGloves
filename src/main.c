
#include "stm32f407xx.h"
#include "adc.h"
#include "UART.h"
#include "DMA.h"
#include "I2C.h"
#include "LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "BNO055.h"
#include "stdio.h"
#include "circular_buffer.h"
#include "clockconfig.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include "CS43L22.h"
#include "I2S.h"
			



			//one is faster than the other -> depends on order and it shouldn't

			
			// TO DO: filter function / use madgwick algorithm/ set I2S / set DAC/

			// circular buffer for ADC data
			
#define BNO055_ADDRESS_A 0x28
#define LSM9DS1_AG_ADDR  0x6B
#define LSM9DS1_M_ADDR  0x1E
#define SENSITIVITY_ACCELEROMETER_2  (float)0.000061
#define SENSITIVITY_GYROSCOPE_245    (float)0.00875
#define SENSITIVITY_MAGNETOMETER_4   (float)0.00014
#define PI 3.1415
#define DECLINATION 21.58

#define LUT_SIZE 128
#define DMA_I2S_BUFFER_SIZE 128


static int16_t lut[] = {3750,3934,4118,4300,4482,4661,4839,5013,
5185,5353,5518,5678,5833,5984,6129,6268,
6402,6529,6649,6762,6868,6966,7057,7140,
7215,7281,7339,7388,7428,7459,7482,7495,
7500,7495,7482,7459,7428,7388,7339,7281,
7215,7140,7057,6966,6868,6762,6649,6529,
6402,6268,6129,5984,5833,5678,5518,5353,
5185,5013,4839,4661,4482,4300,4118,3934,
3750,3566,3382,3200,3018,2839,2661,2487,
2315,2147,1982,1822,1667,1516,1371,1232,
1098,971,851,738,632,534,443,360,
285,219,161,112,72,41,18,5,
0,5,18,41,72,112,161,219,
285,360,443,534,632,738,851,971,
1098,1232,1371,1516,1667,1822,1982,2147,
2315,2487,2661,2839,3018,3200,3382,3566};

static volatile uint16_t DMAI2SBuffer0[128] __attribute__ ((aligned (4)));
static volatile uint16_t DMAI2SBuffer1[128] __attribute__ ((aligned (4)));

volatile uint8_t DMA_I2S_buffer_flag = 0;
volatile uint8_t wait_read = 0;


int8_t adc_counter = 0;
uint16_t result = 0;
char data = 'b';
uint32_t average_result = 0;
uint8_t i2c_counter = 0;
uint8_t i2c_data_counter = 0;
volatile int32_t filtered_flex[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int32_t output_data[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
volatile int16_t buffer[16];
volatile int count = 0;

volatile int16_t imu_data[] = {0x0, 0x0, 0x0};
uint8_t imu_test[] = {0x0, 0x0, 0x1, 0x1, 0x1, 0x1};
volatile int16_t dummy = 0;
volatile uint8_t systick_flag = 0;

volatile float imu_float[3];
int32_t aBias[] = {0, 0, 0};
int32_t gBias[] = {0, 0, 0};
volatile uint8_t AHRS_flag = 0;

volatile uint32_t flag = 0;
volatile uint32_t sr1 = 0;
volatile uint32_t sr2 = 0;

i2c_address i2c_buffer[32];

uint32_t i = 0;
uint32_t j = 0;
volatile uint32_t step = 1;

volatile uint8_t delay_var = 0;
volatile uint8_t delay_time = 0;

void DMA2_Stream0_IRQHandler()
{
	if (((DMA2->LISR)&(DMA_LISR_TCIF0)) == DMA_LISR_TCIF0 )
	{
		systick_flag = 0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LISR_HTIF0;
	}
}

void DMA1_Stream5_IRQHandler()
{
	if (DMA_I2S_buffer_flag == 0)
	{
		DMA_I2S_buffer_flag = 1;
	}
	else if (DMA_I2S_buffer_flag == 1)
	{
		DMA_I2S_buffer_flag = 0;
	}
	wait_read = 0;
	DMA1->HIFCR = DMA_HIFCR_CTCIF5;
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
	if (AHRS_flag == 0)
	{
		AHRS_flag = 1;
	}
	if (delay_var == 1)
	{
		delay_time++;
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

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  roll = atan2(ay, az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  
  if (my == 0)
    yaw = (mx < 0) ? PI : 0;
  else
    yaw = atan2(mx, my);
    
  yaw -= DECLINATION * PI / 180;
  
  if (yaw > PI) yaw -= (2 * PI);
  else if (yaw < -PI) yaw += (2 * PI);
  else if (yaw < 0) yaw += 2 * PI;
  
  // Convert everything from radians to degrees:
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
}


void delay(int time)
{
	delay_var = 1;
	while(delay_time < time/10);
	delay_var = 0;
	delay_time = 0;

}

void bno055_init()
{
	I2C_Write(I2C2, BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x20); // reset
	delay(700);
	I2C_Write(I2C2, BNO055_ADDRESS_A, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL); // set to normal power mode
	delay(30);
	I2C_Write(I2C2, BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x80); // set to use external crystal
	delay(10);
	I2C_Write(I2C2, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF); // set to NDOF mode 
	delay(30);
}



int main(void)
{
	ClockConfig();
	//cs43l22_init();
	//cs43l22_ctrl_config();
	//I2C_Write(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PWR_CTRL1, 0x9E);
	//I2C_test = I2C_Read(I2C1, CODEC_I2C_ADDRESS, CODEC_MAP_PLAYBACK_CTRL1);
	/*i2s_init();
	NVIC_SetPriority(DMA1_Stream5_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	DMA_I2S_config();
	DMA1_Stream5 -> M0AR |= (uint32_t)DMAI2SBuffer0;
	DMA1_Stream5 -> M1AR |= (uint32_t)DMAI2SBuffer1;
	DMA_I2S_config2();*/
	
	USARTclock_config();
	I2C_clock_init();
	I2C_gpio_config();
	I2C_config(I2C2);
	NVIC_SetPriority(I2C2_EV_IRQn, 1);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	GPIO_config();
	USART_config();
	NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_EnableIRQ(ADC_IRQn);
	ADCclock_config();
	GPIOx_config();
	NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA_ADC_config();
	DMA2_Stream0 -> M0AR |= (uint32_t)buffer;
	DMA_ADC_config2();
	adc_config_multi();
	(void)SysTick_Config(0x19A280); //334500 0x19A280
	NVIC_SetPriority(SysTick_IRQn, 3);
	bno055_init();
	I2C2->CR2 |= (I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN);


	for(;;)
	{
		/*if (DMA_I2S_buffer_flag == 0 && wait_read == 0)
		{
			if (j*step >= 128)
			{
				DMAI2SBuffer0[i] = lut[0];
			}
			else
			{
				DMAI2SBuffer0[i] = lut[j*step];
			}
			i++;
			j++;
			if (i == DMA_I2S_BUFFER_SIZE)
			{
				i = 0;
				wait_read = 1;
			}
			if (j*step >= LUT_SIZE)
			{
				j = 0 ;
			}
		}
		else if (DMA_I2S_buffer_flag == 1 && wait_read == 0)
		{
			if (j*step >= 128)
			{
				DMAI2SBuffer1[i] = lut[0];
			}
			else
			{
				DMAI2SBuffer1[i] = lut[j*step];
			}
			i++;
			j++;
			if (i == DMA_I2S_BUFFER_SIZE)
			{
				i = 0;
				wait_read = 1;
			}
			if (j*step >= LUT_SIZE)
			{
				j = 0 ;
		*/
		
		i2c_read_it(I2C2,BNO055_ADDRESS_A, 0x1A, 6, imu_test);
		imu_data[0] = (imu_test[1]<<8) | imu_test[0];
		imu_data[1] = (imu_test[3]<<8) | imu_test[2];
		imu_data[2] = (imu_test[5]<<8) | imu_test[4];

  		imu_float[0] = ((float)imu_data[0])/16.0;
		imu_float[1] = ((float)imu_data[1])/16.0;
		imu_float[2] = ((float)imu_data[2])/16.0;
		
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




		/*send_data('x');
		send_data(' ');
		print_float(imu_float[0]);
		//send_data('\n');
		//send_data('\r');
		//print_data((int32_t)filtered_flex[1]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		print_float(imu_float[1]);
		//send_data('\n');
		//send_data('\r');
		//print_data((int32_t)accel_data[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		print_float(imu_float[2]);*/
		//send_data('\n');
		//send_data('\r');
		//print_data((int32_t)accel_data[2]);


		/*send_data(' ');
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
		print_float(mag_float[0]);
		//print_data((int32_t)mag_data[0]);
		send_data(' ');
		send_data('y');
		send_data(' ');
		print_float(mag_float[1]);
		//print_data((int32_t)mag_data[1]);
		send_data(' ');
		send_data('z');
		send_data(' ');
		print_float(mag_float[2]);
		//print_data((int32_t)mag_data[2]);
		send_data(' ');*/

		send_data('a');
		send_data(' ');
		print_data(filtered_flex[0]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('b');
		send_data(' ');
		print_data(filtered_flex[1]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('c');
		send_data(' ');
		print_data(filtered_flex[2]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('d');
		send_data(' ');
		print_data(filtered_flex[3]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('e');
		send_data(' ');
		print_data(filtered_flex[4]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('f');
		send_data(' ');
		print_data(filtered_flex[5]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('g');
		send_data(' ');
		print_data(filtered_flex[6]);
		//send_data('\n');
		//send_data('\r');
		send_data(' ');
		send_data('h');
		send_data(' ');
		print_data(filtered_flex[7]);
		send_data('\n');
		send_data('\r');
		count++;
		//count = DWT->CYCCNT;
		//-----------------------------
	}
}

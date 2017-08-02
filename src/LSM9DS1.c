#include "stm32f407xx.h"
#include "LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "I2C.h"

#define LSM9DS1_AG_ADDR  0x6B
#define LSM9DS1_M_ADDR  0x1E
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_MAGNETOMETER_4   0.00014

void LSM9DS1_init()
{
	accel_init(6,0);
	gyro_init(6,0);
	mag_init(7);
}


void accel_init(uint8_t ODR, uint8_t bandwidth)
{
	uint8_t tempreg = 0;
	tempreg |= (ODR & 0x7)<<5; // ODR = sample rate 6 different values, see table 68 pg.52 
	tempreg |= (1<<2); // if 1 user can choose bandwidth value
	tempreg |= (bandwidth & 0x03); // 4 different values table 67 pg.52
	I2C_Write(I2C2, LSM9DS1_AG_ADDR, CTRL_REG6_XL, tempreg); // write tempreg to CTRL_REG6_XL


}

uint8_t accel_available()
{
	uint8_t status = I2C_Read(I2C2, LSM9DS1_AG_ADDR, STATUS_REG_1);
	return (status & (1<<0));
}

void accel_read(int16_t *accel_data, int32_t *aBiasRaw)
{
	uint8_t data[6];
	I2C_Read_Many(I2C2, LSM9DS1_AG_ADDR , OUT_X_L_XL|0x80, data, 6);
	accel_data[0] = (data[1]<<8) | data[0];
	accel_data[1] = (data[3]<<8) | data[2];
	accel_data[2] = (data[5]<<8) | data[4];

	accel_data[0] = accel_data[0] - aBiasRaw[0];
	accel_data[1] = accel_data[1] - aBiasRaw[1];
	accel_data[2] = accel_data[2] - aBiasRaw[2];
}

void accel_g(float *accel_data, int32_t *aBiasRaw)
{
	int16_t data[3];
	accel_read(data, aBiasRaw);
	accel_data[0] = SENSITIVITY_ACCELEROMETER_2*data[0];
	accel_data[1] = SENSITIVITY_ACCELEROMETER_2*data[1];
	accel_data[2] = SENSITIVITY_ACCELEROMETER_2*data[2];
}


void gyro_init(uint8_t ODR, uint8_t bandwidth)
{
	uint8_t tempreg = 0;
	tempreg |= (ODR & 0x7)<<5; // ODR = sample rate 6 different values, see Table 46 pg.45
	tempreg |= (bandwidth & 0x3); // 4 different values table 47 pg.46
	I2C_Write(I2C2, LSM9DS1_AG_ADDR, CTRL_REG1_G, tempreg); // write tempreg to CTRL_REG1_G

}

uint8_t gyro_available()
{
	uint8_t status = I2C_Read(I2C2, LSM9DS1_AG_ADDR, STATUS_REG_1);
	return ((status & (1<<1)) >> 1);
}

void gyro_read(int16_t *gyro_data, int32_t *gBiasRaw)
{
	uint8_t data[6];
	I2C_Read_Many(I2C2, LSM9DS1_AG_ADDR , OUT_X_L_G|0x80, data, 6);
	gyro_data[0] = (data[1]<<8) | data[0];
	gyro_data[1] = (data[3]<<8) | data[2];
	gyro_data[2] = (data[5]<<8) | data[4];

	gyro_data[0] = gyro_data[0] - gBiasRaw[0];
	gyro_data[1] = gyro_data[1] - gBiasRaw[1];
	gyro_data[2] = gyro_data[2] - gBiasRaw[2];
}

void gyro_dps(float *gyro_data, int32_t *gBiasRaw)
{
	int16_t data[3];
	gyro_read(data, gBiasRaw);
	gyro_data[0] = SENSITIVITY_GYROSCOPE_245*data[0];
	gyro_data[1] = SENSITIVITY_GYROSCOPE_245*data[1];
	gyro_data[2] = SENSITIVITY_GYROSCOPE_245*data[2];
}

void mag_init(uint8_t ODR)
{
	uint8_t tempreg = 0;
	tempreg |= (3<<5); // 4 levels of performance
	tempreg |= (ODR & 0x7)<<2; // 7 different sample rates values, see Table 111 pg.63
	I2C_Write(I2C2, LSM9DS1_M_ADDR, CTRL_REG1_M, tempreg);

	tempreg = 0;
	I2C_Write(I2C2, LSM9DS1_M_ADDR, CTRL_REG3_M, tempreg);

	tempreg = 0;
	tempreg |= 3<<2;
	I2C_Write(I2C2, LSM9DS1_M_ADDR, CTRL_REG4_M, tempreg);

}

uint8_t mag_available()
{
	uint8_t status = I2C_Read(I2C2, LSM9DS1_M_ADDR, STATUS_REG_M);
	return ((status & (1<<3)) >> 3);
}

void mag_read(int16_t *mag_data)
{
	uint8_t data[6];
	I2C_Read_Many(I2C2, LSM9DS1_M_ADDR, OUT_X_L_M|0x80, data, 6);
	mag_data[0] = (data[1]<<8) | data[0];
	mag_data[1] = (data[3]<<8) | data[2];
	mag_data[2] = (data[5]<<8) | data[4];
}

void mag_gs(float *mag_data)
{
	int16_t data[3];
	mag_read(data);
	mag_data[0] = SENSITIVITY_MAGNETOMETER_4*data[0];
	mag_data[1] = SENSITIVITY_MAGNETOMETER_4*data[1];
	mag_data[2] = SENSITIVITY_MAGNETOMETER_4*data[2];
}

void accel_gyro_calibrate(int32_t *aBiasRaw, int32_t *gBiasRaw)
{
	uint8_t samples = 0;
	int ii;
	int16_t aBiasRawTemp[3] = {0, 0, 0};
	int16_t gBiasRawTemp[3] = {0, 0, 0};

	// Turn on FIFO and set threshold to 32 samples
	enable_FIFO(1);
	set_FIFO(1,0x1F);

	while(samples < 0x1F)
	{
		samples = (I2C_Read(I2C2, LSM9DS1_AG_ADDR, FIFO_SRC) & 0x3F); // Read number of stored samples
	}

	for(ii = 0; ii < samples ; ii++) 
	{	// Read the gyro data stored in the FIFO
		gyro_read(gBiasRawTemp, gBiasRaw);
		accel_read(aBiasRawTemp, aBiasRaw);
		aBiasRawTemp[2] = aBiasRawTemp[2] - (int16_t)(1./SENSITIVITY_ACCELEROMETER_2); // Assumes sensor facing up!
	}  

	for (ii = 0; ii < 3; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		//gBias[ii] = gBiasRaw[ii]*SENSITIVITY_GYROSCOPE_245;
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		//aBias[ii] = aBiasRaw[ii]*SENSITIVITY_ACCELEROMETER_2;
	}

	enable_FIFO(0);
	set_FIFO(0,0x00);

}

void enable_FIFO(uint8_t enable)
{
	uint8_t temp = I2C_Read(I2C2, LSM9DS1_AG_ADDR, CTRL_REG9);
	if (enable == 1)
	{
		temp |= (1<<1);
	}
	else
	{
		temp &= ~(1<<1);
	}
	I2C_Write(I2C2, LSM9DS1_AG_ADDR, CTRL_REG9, temp);
}

void set_FIFO(uint8_t fifo_mode, uint8_t fifo_ths)
{
	I2C_Write(I2C2, LSM9DS1_AG_ADDR, FIFO_CTRL, ((fifo_mode & 0x7) << 5) | (fifo_ths & 0x1F));
}

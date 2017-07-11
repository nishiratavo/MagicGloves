#include "stm32f407xx.h"
#include "LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "I2C.h"

#define LSM9DS1_AG_ADDR  0x6B


void LSM9DS1_init()
{
	accel_init(6,0);
	initGyro();
	initMag();
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

void accel_read(uint16_t *accel_data)
{
	uint8_t data[6];
	I2C_Read_Many(I2C2, LSM9DS1_AG_ADDR , OUT_X_L_XL|0x80, data, 6);
	accel_data[0] = (data[1]<<8) | data[0];
	accel_data[1] = (data[3]<<8) | data[2];
	accel_data[2] = (data[5]<<8) | data[4];

}

void initGyro()
{
	
}


void initMag()
{
	
}
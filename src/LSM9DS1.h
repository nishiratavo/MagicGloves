#ifndef LSM9DS1_H_
#define LSM9DS1_H_

void LSM9DS1_init();
void accel_init(uint8_t ODR, uint8_t bandwidth);
uint8_t accel_available();
void accel_read(uint16_t *accel_data);
void initGyro();
void initMag();






#endif
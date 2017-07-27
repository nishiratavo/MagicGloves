#ifndef LSM9DS1_H_
#define LSM9DS1_H_

void LSM9DS1_init();
void accel_init(uint8_t ODR, uint8_t bandwidth);
uint8_t accel_available();
void accel_read(int16_t *accel_data);
void accel_g(float *accel_data);
void gyro_init(uint8_t ODR, uint8_t bandwidth);
uint8_t gyro_available();
void gyro_read(int16_t *gyro_data);
void gyro_dps(float *gyro_data);
void mag_init(uint8_t ODR);
uint8_t mag_available();
void mag_read(int16_t *mag_data);
void mag_gs(float *mag_data);






#endif
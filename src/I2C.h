#ifndef I2C_H_
#define I2C_H_

void I2C_clock_init();
void I2C_gpio_config();
void I2C_acknowledge(char config);
uint8_t I2C_check_event(uint32_t event);
void I2C_config(I2C_TypeDef* I2Cx);
void I2C_busy_errata();
void I2C_start(I2C_TypeDef* I2Cx, uint8_t adress, char direction, char check_busy);
void I2C_write(uint8_t data);
uint8_t I2C_read_ack();
uint8_t I2C_read_nack();
void I2C_stop();
uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD);
void I2C_Write(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t data);



#endif
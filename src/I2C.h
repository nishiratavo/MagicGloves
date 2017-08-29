#ifndef I2C_H_
#define I2C_H_

#define MASTER_MODE_SELECT  ((uint32_t)0x00030001)
#define MASTER_TRANSMITTER_MODE  ((uint32_t)0x00070082)
#define MASTER_RECEIVER_MODE ((uint32_t)0x00030002)
#define MASTER_BYTE_TRANSMITTED ((uint32_t)0x00070084)
#define MASTER_BYTE_RECEIVED ((uint32_t)0x00030040)

extern volatile uint8_t start_flag;
extern volatile uint8_t master_mode_select;
extern volatile uint8_t master_transmitter_mode;
extern volatile uint8_t master_receiver_mode;
extern volatile uint8_t master_byte_transmitted;
extern volatile uint8_t master_byte_received;
extern volatile uint8_t done_flag;
extern volatile uint8_t i2c_collector;
extern volatile uint8_t i2c_supplier;
extern volatile uint8_t i2c_isr_free;
extern volatile uint8_t first_time;

typedef struct i2c_address
{
	uint8_t SAD;
	uint8_t RAD;
	uint8_t counter;
	uint8_t *pointer_data;
} i2c_address;

extern i2c_address i2c_buffer[32];

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
void I2C_Read_Many(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t *data, uint8_t count);
void I2C_Write(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t data);
void I2C_Read_IT(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t data);
void I2C_write_buffer(i2c_address *buffer, i2c_address *data);
void I2C_read_buffer(i2c_address *buffer, i2c_address *data);
void i2c_read_it(I2C_TypeDef* I2Cx, uint8_t SAD, uint8_t RAD, uint8_t counter, uint8_t *data);
#endif
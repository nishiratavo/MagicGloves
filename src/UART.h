#ifndef UART_H_
#define UART_H_

void USARTclock_config();
void GPIO_config();
void USART_config();
void send_data(char data);
void convert_data(char *converted_data, volatile int32_t value);
void print_data(volatile int32_t data);
uint8_t return_float_index(char *data);
void print_float(float data);


#endif
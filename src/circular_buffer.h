#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_


#define BUFFER_LENGTH 32

extern uint8_t buffer_full;
extern uint8_t buffer_empty;

typedef struct circular_buffer
{
	volatile uint8_t supplier;
	volatile uint8_t collector;
	volatile uint8_t data[BUFFER_LENGTH];

} circular_buffer;

void write_buffer(circular_buffer *buffer, uint8_t *data, uint8_t counter);
void read_buffer(circular_buffer *buffer, uint8_t *data, uint8_t counter);

#endif
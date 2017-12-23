#include "stm32f407xx.h"
#include "circular_buffer.h"

uint8_t buffer_full = 0;
uint8_t buffer_empty = 0;

void write_buffer(circular_buffer *buffer, uint8_t *data, uint8_t counter)
{
	uint8_t current_index = buffer->supplier;
	if (current_index == (buffer->collector - 1)) 
	{
		buffer_full++;
		return;
	}
	uint8_t index;
	for (uint8_t i = 0; i < counter; ++i)
	{
		index = (i + current_index) & (BUFFER_LENGTH -1);
		buffer->data[index] = data[i];
	}
	buffer->supplier += counter;
	buffer->supplier &= (BUFFER_LENGTH -1); 
}

void read_buffer(circular_buffer *buffer, uint8_t *data, uint8_t counter)
{
	uint8_t current_index = buffer->collector;
	if (current_index == buffer->supplier) 
	{
		buffer_empty++;
		return;
	}
	uint8_t index;
	for (int i = 0; i < counter; ++i)
	{
		index = (i + current_index) & (BUFFER_LENGTH -1);
		data[i] = buffer->data[index];
	}
	buffer->collector += counter;
	buffer->collector &= (BUFFER_LENGTH -1); 
}
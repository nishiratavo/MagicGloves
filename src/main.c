/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f407xx.h"
#include "adc.h"
			
uint16_t result = 0;
int main(void)
{
	clock_config();
	GPIOx_config();
	adc_config();
	for(;;)
	{
		result = adc_value();
	}
}

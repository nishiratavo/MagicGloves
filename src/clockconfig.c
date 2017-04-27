#include "clockconfig.h"

#define RCC_PLLCFGR_PLLM_Pos 0
#define RCC_PLLCFGR_PLLN_Pos 6
#define RCC_PLLCFGR_PLLP_Pos 16

int32_t ClockConfig()
{
	/*
	 * Sets the recomended flash memory latency
	 * for HCLK = 168MHz
	 */
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN;

	/*
	 * Enable the high speed external oscillator
	 */
	RCC->CR |= RCC_CR_HSEON;

	/*
	 * Waits until the external oscillator is ready
	 */
	while((RCC->CR & RCC_CR_HSERDY) == 0);

	/*
	 * PLL Source: HSE
	 * PLLM: 4
	 * PLLN: 168
	 * PLLP: 2
	 * AHB Prescaler: 1
	 * APB1 Prescaler: 4
	 * APB2 Prescaler: 2
	 */
	RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV2 |
		RCC_CFGR_PPRE1_DIV4;
	RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (4 << RCC_PLLCFGR_PLLM_Pos) |
		(168 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos);

	/*
	 * Enable the main PLL and waits until it is
	 * locked
	 */
	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/*
	 * Selects the main PLL as the HCLK source
	 */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/*
	 * Waits until the main PLL is selected as the
	 * HCLK source
	 */
	while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

	/*
	 * Updates the clock information
	 */
	SystemCoreClockUpdate();

	return 0;
}

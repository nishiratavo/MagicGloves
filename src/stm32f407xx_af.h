/*
 * GPIO alternative functions definitions
 */

#define AF_SYSTEM	0
#define AF_TIM1		1
#define AF_TIM2		1
#define AF_TIM3		2
#define AF_TIM4		2
#define AF_TIM5		2
#define AF_TIM8		3
#define AF_TIM9		3
#define AF_TIM10	3
#define AF_TIM11	3
#define AF_I2C1		4
#define AF_I2C2		4
#define AF_I2C3		4
#define AF_SPI1		5
#define AF_SPI2		5
#define AF_SPI3		6
#define AF_USART1	7
#define AF_USART2	7
#define AF_USART3	7
#define AF_USART4	8
#define AF_USART5	8
#define AF_USART6	8
#define AF_CAN1		9
#define AF_CAN2		9
#define AF_TIM12	9
#define AF_TIM13	9
#define AF_TIM14	9
#define AF_OTG_FS	10
#define AF_OTG_HS	10
#define AF_ETH		11
#define AF_FSMC		12
#define AF_SDIO		12
#define AF_OTG_HS1	12
#define AF_DCMI		13
#define AF_EVENTOUT	15

#define GPIO_AFRH_AFRH0_Pos	0
#define GPIO_AFRH_AFRH1_Pos	4
#define GPIO_AFRH_AFRH2_Pos	8
#define GPIO_AFRH_AFRH3_Pos	12
#define GPIO_AFRH_AFRH4_Pos	16
#define GPIO_AFRH_AFRH5_Pos	20
#define GPIO_AFRH_AFRH6_Pos	24
#define GPIO_AFRH_AFRH7_Pos	28

#define GPIO_AFRL_AFRL0_Pos	0
#define GPIO_AFRL_AFRL1_Pos	2
#define GPIO_AFRL_AFRL2_Pos	8
#define GPIO_AFRL_AFRL3_Pos	12
#define GPIO_AFRL_AFRL4_Pos	16
#define GPIO_AFRL_AFRL5_Pos	20
#define GPIO_AFRL_AFRL6_Pos	24
#define GPIO_AFRL_AFRL7_Pos	28

#define GPIO_MODER_GPIN		0
#define GPIO_MODER_GPOUT	1
#define GPIO_MODER_AF		2
#define GPIO_MODER_ANALOG	3

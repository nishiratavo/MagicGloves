// DMA configuration 

//Developer : Gustavo Nishihara

#include "stm32f407xx.h"
#include "DMA.h"


void DMA_ADC_clock()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
}

void DMA_I2S_config()
{
	//clock init
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	DMA1_Stream5 -> CR &= ~(DMA_SxCR_EN); // needs to be 0 to set register CR
	DMA1_Stream5 -> CR &= ~(DMA_SxCR_CHSEL); //channel 0
	DMA1_Stream5 -> CR |= DMA_SxCR_DBM; // enables double buffer mode
	DMA1_Stream5 -> NDTR |= 0x80; //(DMA_SxNDT_1 | DMA_SxNDT_0); // buffer size 
	DMA1_Stream5 -> CR |= DMA_SxCR_DIR_0; // memory to peripheral
	DMA1_Stream5 -> FCR |= DMA_SxFCR_DMDIS; // FIFO disable    // certo
	DMA1_Stream5 -> FCR &= ~(DMA_SxFCR_FTH);
	DMA1_Stream5 -> CR &= ~(DMA_SxCR_MBURST); //memory single burst
	DMA1_Stream5 -> CR &= ~(DMA_SxCR_PBURST); // peripheral single burst
	DMA1_Stream5 -> CR |= DMA_SxCR_CIRC; // circular mode
	DMA1_Stream5 -> CR |= DMA_SxCR_PL_1; // priority level high
	DMA1_Stream5 -> CR |= DMA_SxCR_CT; // start with buffer 1 

}

void DMA_I2S_config2()
{
	DMA1_Stream5 -> CR |= DMA_SxCR_MSIZE_0; // memory size 16 bits.   // certo
	DMA1_Stream5 -> CR |= DMA_SxCR_MINC; // increment base address.  // certo
	DMA1_Stream5 -> PAR |= (uint32_t)&SPI3->DR; 
	DMA1_Stream5 -> CR |= DMA_SxCR_PSIZE_0; // peripheral size 16 bits. // certo
	DMA1_Stream5 -> CR &= ~(DMA_SxCR_PINC); // don't increment base address // certo
	//DMA1_Stream5 -> CR |= DMA_SxCR_TEIE; // transfer error interrupt enable
	//DMA1_Stream5 -> CR |= DMA_SxCR_HTIE; // half transfer interrupt enable
	DMA1_Stream5 -> CR |= DMA_SxCR_TCIE; // transfer complete interrupt enable
	DMA1_Stream5 -> CR |= DMA_SxCR_EN; // enable DMA. // certo
}



void DMA_ADC_config()
{
	DMA_ADC_clock();
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_EN); // needs to be 0 to set register CR
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_CHSEL); //channel 0
	DMA2_Stream0 -> NDTR |= 0x10; //(DMA_SxNDT_1 | DMA_SxNDT_0); // buffer size 
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_DIR); // peripheral to memory 
	DMA2_Stream0 -> FCR |= DMA_SxFCR_DMDIS; // FIFO disable    // certo
	DMA2_Stream0 -> FCR &= ~(DMA_SxFCR_FTH);
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_MBURST); //memory single burst
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_PBURST); // peripheral single burst
	DMA2_Stream0 -> CR |= DMA_SxCR_CIRC; // circular mode
	DMA2_Stream0 -> CR |= DMA_SxCR_PL_1; // priority level high

}

void DMA_ADC_config2()
{
	DMA2_Stream0 -> CR |= DMA_SxCR_MSIZE_0; // memory size 16 bits.   // certo
	DMA2_Stream0 -> CR |= DMA_SxCR_MINC; // increment base address.  // certo
	DMA2_Stream0 -> PAR |= (uint32_t)&ADC1->DR; 
	DMA2_Stream0 -> CR |= DMA_SxCR_PSIZE_0; // peripheral size 16 bits. // certo
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_PINC); // don't increment base address // certo
	//DMA2_Stream0 -> CR |= DMA_SxCR_TEIE; // transfer error interrupt enable
	//DMA2_Stream0 -> CR |= DMA_SxCR_HTIE; // half transfer interrupt enable
	DMA2_Stream0 -> CR |= DMA_SxCR_TCIE; // transfer complete interrupt enable
	DMA2_Stream0 -> CR |= DMA_SxCR_EN; // enable DMA. // certo
}
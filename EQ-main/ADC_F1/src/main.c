/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "ADC_NUCLEO.h"



int main(void)
{
	SystemCoreClock = 8000000;	// taktowanie 8Mhz
	HAL_Init();
	UART_Init();
	ADC_Init();
	SPI_init();

	while (1)
	{
		uint8_t adc1_0,adc1_1,adc1_4,adc1_8;
		adc1_0 =  adc_read(ADC_CHANNEL_0)/16;
		adc1_1 =  adc_read(ADC_CHANNEL_1)/16;
		adc1_4 =  adc_read(ADC_CHANNEL_4)/16;
		adc1_8 =  adc_read(ADC_CHANNEL_8)/16;


		 printf("ADC0 = %d ", adc1_0);
		 printf("ADC1 = %d ", adc1_1);
		 printf("ADC4 = %d ", adc1_4);
		 printf("ADC8 = %d\n ", adc1_8);

		HAL_Delay(50);
		SPI_WRITE_ADC(adc1_0,adc1_1,adc1_4,adc1_8);
	}
}

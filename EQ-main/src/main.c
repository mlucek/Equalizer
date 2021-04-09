/**
  ******************************************************************************
  * @file    main.c
  * @author  Lucjan Maciej, Zachariasz Monka
  * @brief   main function.
  ******************************************************************************
*/

#include"layers_port.h"
#include"VS1003b.h"
#include <signals_z.h>

uint16_t s0[2000];
uint16_t s1[2000];
uint16_t s2[2000];
uint16_t s3[2000];
uint16_t s4[2000];
uint16_t lo_u[2000];
uint16_t lo_i[2000];
uint16_t hi_u[2000];
uint16_t hi_i[2000];
uint16_t input[2000];
uint16_t input_i[2000];
uint16_t output[2000];

int main()
{
	LP_init();
	VS1003b_Init();
	syg_init();

	VS1003b_Play_48kHz_Init();


	//READ  LED	wait for ADC
	//GREEN LED wait for DAC
	//BLUE  LED data processing
	while(1)
	{
		LP_LED(LP_LED_RED,LP_LED_OFF);
		LP_ADC_wait_FULL();
		LP_LED(LP_LED_RED,LP_LED_ON);
		LP_ADC_read(input,2000);

		LP_LED(LP_LED_BLUE,LP_LED_ON);
		uint8_t Volium;
		uint8_t LowB;
		uint8_t HighB;
		LowB = LP_ADC_EXTERNAL_CH_L();
		HighB = LP_ADC_EXTERNAL_CH_H();
		Volium = LP_ADC_EXTERNAL_CH_V();

		int16_t fir1[8] = {10,10,10,10,10,10,10,10};
		int16_t fir2[8] = {2,-6,10,-11,8,-3,1,0};

		syg_uint_to_int(input,input_i,2000);
		syg_FIR(input_i,lo_i,2000,fir1);
		syg_FIR(input_i,hi_i,2000,fir2);
		syg_int_to_uint(lo_i,lo_u,2000);
		syg_int_to_uint(hi_i,hi_u,2000);

		syg_mix2(lo_u,LowB,hi_u,HighB,s1,2000);
		syg_mix2(s1,Volium,s1,Volium,s3,2000);

		syg_decoding(s3,output,2000);
		LP_LED(LP_LED_BLUE,LP_LED_OFF);



		LP_LED(LP_LED_GREEN,LP_LED_OFF);
		LP_VS1003_WRITE_DATA_wait_for_end();
		LP_LED(LP_LED_GREEN,LP_LED_ON);
		VS1003b_Play(output,2000);
	}
}




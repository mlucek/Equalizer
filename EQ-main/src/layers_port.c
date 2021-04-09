/*
 * layers_port.c
 *
 * @author  Lucjan Maciej, Zachariasz Monka
 *
 */

#include "layers_port.h"



void SystemClock_Config(void)
{
	#define SYSCLK_CLOCK 96000000UL
	#define HCLK_CLOCK 96000000UL
	#define ETHERNET_PTP_CLOCK 96000000UL
	#define SYSTEM_TIMER_CLOCK 96000000UL
	#define FCLK_CLOCK 96000000UL
	#define APD1_PERIPH_CLOCK 48000000UL
	#define APD1_TIMER_CLOCK 96000000UL
	#define APD2_PERIPH_CLOCK 96000000UL
	#define APD2_TIMER_CLOCK 96000000UL
	#define I2S_CLOCK 48000000UL

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_SYSTICK_Config(SYSCLK_CLOCK/1000UL) != HAL_OK)
	{
		Error_Handler();
	}
}
void Error_Handler(void)
{

}
void LP_Delay(uint32_t Delay)
{
	//idea sleep MCU
	HAL_Delay(Delay);
}

volatile void LLP_interrup_EXTI1(void)
{
	if(HAL_GPIO_ReadPin(LP_port_RES, LP_pin_DREQ)==GPIO_PIN_RESET)
	{//Wati
		HAL_SPI_DMAPause(&LLP_hspi4);
	}
	if(HAL_GPIO_ReadPin(LP_port_RES, LP_pin_DREQ)==GPIO_PIN_SET)
	{//Resume
		HAL_SPI_DMAResume(&LLP_hspi4);
	}

}

volatile void LLP_interrup_EXTI3(void)
{
	if(HAL_GPIO_ReadPin(LP_port_SPI_ADC_CS, LP_pin_SPI_ADC_CS)==GPIO_PIN_RESET)
	{
		HAL_SPI_Receive(&LLP_hspi5, LP_ADC_EXTERNAL, 4, 2);
	}


}
volatile void LLP_iunerrup_dma_tx(void)
{
	if(HAL_DMA_GetState(&LLP_dma_spi4_tx)==HAL_DMA_STATE_READY)
	{
		LLP_SPI_CS_SCI_active_SDI_inactive();
	}
	//if(HAL_DMA_GetState(&LLP_dma_spi4_tx)==HAL_DMA_STATE_BUSY){}//half

}
/*
volatile void LLP_iunerrup_tim10(void)
{
	if(LLP_tim10_cycle++ == 5)
	{
		LLP_tim10_cycle = 0;

		if(LLP_led_tab[LP_LED_BLUE] == LP_LED_BLINK_1Hz)
			HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_BLUE);
		if(LLP_led_tab[LP_LED_GREEN] == LP_LED_BLINK_1Hz)
			HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_GREEN);
		if(LLP_led_tab[LP_LED_ORANGE] == LP_LED_BLINK_1Hz)
			HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_ORANGE);
		if(LLP_led_tab[LP_LED_RED] == LP_LED_BLINK_1Hz)
			HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_RED);
	}

	if(LLP_led_tab[LP_LED_BLUE] == LP_LED_BLINK_5Hz)
		HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_BLUE);
	if(LLP_led_tab[LP_LED_GREEN] == LP_LED_BLINK_5Hz)
		HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_GREEN);
	if(LLP_led_tab[LP_LED_ORANGE] == LP_LED_BLINK_5Hz)
		HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_ORANGE);
	if(LLP_led_tab[LP_LED_RED] == LP_LED_BLINK_5Hz)
		HAL_GPIO_TogglePin(LP_port_LED, LP_pin_LED_RED);
}
*/
void LP_init(void)
{
	HAL_Init();
	SystemClock_Config();
	//initialization peripheries using in port_layers library

	//todo NVIC pioryty conf

	//GPIO //todo external function
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

		//LEDs
	LLP_gpio.Pin = LP_pin_LED_BLUE | LP_pin_LED_GREEN
			| LP_pin_LED_ORANGE | LP_pin_LED_RED;
	LLP_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	LLP_gpio.Pull = GPIO_NOPULL;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_LED, &LLP_gpio);
		//VS1003
	LLP_gpio.Pin = LP_pin_SCK|LP_pin_MISO|LP_pin_MOSI;
	LLP_gpio.Mode = GPIO_MODE_AF_PP;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	LLP_gpio.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(LP_port_SPI, &LLP_gpio);

	LLP_gpio.Pin = LP_pin_DCS | LP_pin_CCS| LP_pin_RES;
	LLP_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	LLP_gpio.Pull = GPIO_NOPULL;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_RES, &LLP_gpio);

	LLP_gpio.Pin = LP_pin_DREQ;
	LLP_gpio.Mode = GPIO_MODE_INPUT;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_RES, &LLP_gpio);
		//ADC
	LLP_gpio.Mode = GPIO_MODE_ANALOG;
	LLP_gpio.Pin = LP_pin_ADC;
	HAL_GPIO_Init(LP_port_ADC, &LLP_gpio);

		//SPI to ADC //todo define for pin and port
	LLP_gpio.Pin = GPIO_PIN_0|GPIO_PIN_8; //sck, mosi
	LLP_gpio.Mode = GPIO_MODE_AF_PP;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	LLP_gpio.Alternate = GPIO_AF6_SPI5;
	HAL_GPIO_Init(GPIOB, &LLP_gpio);

	LLP_gpio.Pin = GPIO_PIN_12; //miso
	LLP_gpio.Mode = GPIO_MODE_AF_PP;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	LLP_gpio.Alternate = GPIO_AF6_SPI5;
	HAL_GPIO_Init(GPIOA, &LLP_gpio);

	LLP_gpio.Pin = LP_pin_SPI_ADC_CS;
	LLP_gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_SPI_ADC_CS, &LLP_gpio);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	//GPIO END

	//VARIABLE
	LLP_ADC_tab = LLP_ADC_READY;
	LLP_led_tab[LP_LED_BLUE]   = LP_LED_OFF;
	LLP_led_tab[LP_LED_RED]    = LP_LED_OFF;
	LLP_led_tab[LP_LED_ORANGE] = LP_LED_OFF;
	LLP_led_tab[LP_LED_GREEN]  = LP_LED_OFF;
	LLP_tim10_cycle = 0;
	//VARIABLE END

	//timer 10 (100ms)
	//todo external function
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	__HAL_RCC_TIM10_CLK_ENABLE();

	LLP_tim10.Instance = TIM10;
	LLP_tim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; //96MHz
	LLP_tim10.Init.Prescaler = 10-1;//48000-1; //2kHz
	LLP_tim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	LLP_tim10.Init.Period = 2*100-1;//48kHz //2*100-1; //10Hz
	LLP_tim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&LLP_tim10);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_TIM_Base_Start_IT(&LLP_tim10);
	//end timer 10



	LP_SPI_low_speed();
	LLP_DMA_init();
	LLP_ADC_init();
	//LLP_TIM2_init();
	LP_SPI_ADC();



}

void LP_SPI_ADC(void)
{
	__HAL_RCC_SPI5_CLK_ENABLE();

	LLP_hspi5.Instance = SPI5;
	LLP_hspi5.Init.Mode = SPI_MODE_SLAVE;
	LLP_hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	LLP_hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
	LLP_hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	LLP_hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	LLP_hspi5.Init.NSS = SPI_NSS_SOFT;
	//LLP_hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;//todo?
	LLP_hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	LLP_hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	LLP_hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	LLP_hspi5.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&LLP_hspi5) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_SPI_ENABLE(&LLP_hspi5);

}



void LP_LED(LP_LED_COLOR color, LP_LED_STATUS status)
{
	//eg. LP_LED(LP_LED_ORANGE, LP_LED_ON);

	LLP_led_tab[color] = status;
	uint16_t pin;
	switch (color)
	{
		case LP_LED_BLUE:
			pin = LP_pin_LED_BLUE;
			break;
		case LP_LED_RED:
			pin = LP_pin_LED_RED;
			break;
		case LP_LED_ORANGE:
			pin = LP_pin_LED_ORANGE;
			break;
		case LP_LED_GREEN:
			pin = LP_pin_LED_GREEN;
			break;
	}

	if(status == LP_LED_TOGLE)
	{
		HAL_GPIO_TogglePin(LP_port_LED, pin);
	}
	else if(status == LP_LED_ON)
	{
		HAL_GPIO_WritePin(LP_port_LED, pin, SET);
	}
	else
	{
		HAL_GPIO_WritePin(LP_port_LED, pin, RESET);
	}
}
void LP_SPI_low_speed(void)
//750kbps
{
	__HAL_RCC_SPI4_CLK_ENABLE();

	LLP_hspi4.Instance = SPI4;
	LLP_hspi4.Init.Mode = SPI_MODE_MASTER;
	LLP_hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	LLP_hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
	LLP_hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	LLP_hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	LLP_hspi4.Init.NSS = SPI_NSS_SOFT;
	LLP_hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	LLP_hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	LLP_hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	LLP_hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	LLP_hspi4.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&LLP_hspi4) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_SPI_ENABLE(&LLP_hspi4);

}
void LP_SPI_high_speed(void)
//8Mbps
{
	__HAL_RCC_SPI4_CLK_ENABLE();

	LLP_hspi4.Instance = SPI4;
	LLP_hspi4.Init.Mode = SPI_MODE_MASTER;
	LLP_hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	LLP_hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
	LLP_hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	LLP_hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	LLP_hspi4.Init.NSS = SPI_NSS_SOFT;
	LLP_hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	LLP_hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	LLP_hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	LLP_hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	LLP_hspi4.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&LLP_hspi4) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_SPI_ENABLE(&LLP_hspi4);

}
uint16_t LP_VS1003_register_read(uint8_t register_adres)
{
	//example use
	//output[0] = LP_VS1003_register_read(0);


	uint16_t read_mode = 0b00000011;

	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	uint16_t send_msb,send_lsb;
	send_msb = (read_mode)<<8;
	send_lsb = (register_adres)<<0;
	uint16_t send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	uint16_t out;
	LLP_SPI_read(&out,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
	return out;
}
void LP_VS1003_register_write(uint8_t register_adres, uint16_t data)
{
	//example use
	//LP_VS1003_register_write(3,0xC430);

	uint16_t write_mode = 0b00000010;

	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	uint16_t send_msb,send_lsb;
	send_msb = (write_mode)<<8;
	send_lsb = (register_adres)<<0;
	uint16_t send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	LLP_SPI_write(&data,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
}
void LP_VS1003_set_bit(uint8_t register_adres, uint16_t bit)
{
	uint8_t read_mode = 0b00000011;
	uint8_t write_mode = 0b00000010;
	uint16_t buf;
	uint16_t send_msb,send_lsb;
	uint16_t send;
	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	send_msb = (read_mode)<<8;
	send_lsb = (register_adres)<<0;
	send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	LLP_SPI_read(&buf,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
	buf |= bit;
	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	send_msb = (write_mode)<<8;
	send_lsb = (register_adres)<<0;
	send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	LLP_SPI_write(&buf,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
}
void LP_VS1003_reset_bit(uint8_t register_adres, uint16_t bit)
{
	uint8_t read_mode = 0b00000011;
	uint8_t write_mode = 0b00000010;
	uint16_t buf;
	uint16_t send_msb,send_lsb;
	uint16_t send;
	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	send_msb = (read_mode)<<8;
	send_lsb = (register_adres)<<0;
	send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	LLP_SPI_read(&buf,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
	buf &= ~bit;
	LLP_DREQ_WAIT();
	LLP_SPI_CS_SCI_active_SDI_inactive();
	send_msb = (write_mode)<<8;
	send_lsb = (register_adres)<<0;
	send = send_msb|send_lsb;
	LLP_SPI_write(&send,1);
	LLP_SPI_write(&buf,1);
	LLP_SPI_CS_SCI_inactive_SDI_active();
}
void LP_VS1003_WRITE_DATA_pooling(uint16_t* data,uint16_t size)
{
	HAL_SPI_DMAStop(&LLP_hspi4);
	uint16_t cycle = 0;
	while(size > cycle)
	{
		LLP_SPI_CS_SCI_inactive_SDI_active();
		LLP_DREQ_WAIT();  //wait to permission to send data
		if(size - cycle >=16)
		{
			LLP_SPI_write(data + cycle,16);
			cycle+=16;
		}
		else
		{
			LLP_SPI_write(data + cycle,1);
			cycle+=1;
		}
		LLP_SPI_CS_SCI_active_SDI_inactive();
	}
}
void LP_VS1003_WRITE_DATA(uint16_t* data,uint16_t size)
{
	//set data to send ane do NOT wait for end
	//remember use LP_VS1003_WRITE_DATA_wait_for_end(); after this
	//example code:
	//uint16_t buff[] ={1,2,3,4,5};
	//LP_VS1003_WRITE_DATA(buff,sizeof(buff)/2);
	//LP_VS1003_WRITE_DATA_wait_for_end();

	HAL_SPI_DMAStop(&LLP_hspi4);
	LLP_SPI_CS_SCI_inactive_SDI_active();
	LLP_DREQ_WAIT();
	LLP_DREQ_mode_interrup();
	HAL_SPI_Transmit_DMA(&LLP_hspi4,data,size);
}
void LP_VS1003_WRITE_DATA_wait_for_end(void)
{
	while(HAL_DMA_GetState(&LLP_dma_spi4_tx)==HAL_DMA_STATE_BUSY){}
}
void LP_VS1003_Hardware_reset(void)
{
	LLP_SPI_RES_active();
	LP_Delay(5);
	LLP_SPI_RES_inactive();
	LP_Delay(5);
	LP_VS1003_register_read(0); //first data is lost
}
void LP_ADC_read(uint16_t* data,uint16_t size)
{
	//example
	//uint16_t adc[99];
	//LP_ADC_read(adc,99);
	//LP_ADC_wait_FULL();
	HAL_ADC_Stop_DMA(&LP_ADC);
	LLP_ADC_tab = LLP_ADC_WAIT;
	HAL_ADC_Start_DMA(&LP_ADC, data,size);
}
void LP_ADC_wait_FULL(void)
{
	while(LLP_ADC_tab != LLP_ADC_READY)
	{}
}
void LP_ADC_wait_HALF(void)
{
	while(LLP_ADC_tab != LLP_ADC_HALF)
	{}
}
uint8_t LP_ADC_EXTERNAL_CH_L(void)
{
	return LP_ADC_EXTERNAL[0];
}
uint8_t LP_ADC_EXTERNAL_CH_M(void)
{
	return LP_ADC_EXTERNAL[1];
}
uint8_t LP_ADC_EXTERNAL_CH_H(void)
{
	return LP_ADC_EXTERNAL[2];
}
uint8_t LP_ADC_EXTERNAL_CH_V(void)
{
	return LP_ADC_EXTERNAL[3];
}

void LLP_TIM2_init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
	tim2.Instance = TIM2;
	tim2.Init.Period = 900;
	tim2.Init.Prescaler = 1990;
	tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.RepetitionCounter = 0;
	tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&tim2);

	TIM_ClockConfigTypeDef timclk;
	timclk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&tim2,&timclk);

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&tim2, &sMasterConfig);
	HAL_TIM_Base_Start(&tim2);
}
void LLP_ADC_init(void)
{
	__HAL_RCC_ADC1_CLK_ENABLE();

	LP_ADC.Instance = ADC1;
	LP_ADC.Init.ContinuousConvMode = DISABLE;
	LP_ADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;//ADC_EXTERNALTRIGCONV_T2_TRGO;
	LP_ADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_RISING;
	LP_ADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	LP_ADC.Init.ScanConvMode = DISABLE;
	LP_ADC.Init.NbrOfConversion = 1;
	LP_ADC.Init.NbrOfDiscConversion = 1;
	LP_ADC.Init.DiscontinuousConvMode = DISABLE;
	LP_ADC.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; //25MHZ
	LP_ADC.Init.Resolution = ADC_RESOLUTION12b; //1600kHz
	LP_ADC.Init.DMAContinuousRequests = ENABLE;
	HAL_ADC_Init(&LP_ADC);

	//todo calibration
	ADC_ChannelConfTypeDef LP_ADC_CH;
	LP_ADC_CH.Channel = ADC_CHANNEL_1;
	LP_ADC_CH.Rank = 1;
	LP_ADC_CH.SamplingTime = ADC_SAMPLETIME_3CYCLES; //533kHz
	HAL_ADC_ConfigChannel(&LP_ADC, &LP_ADC_CH);

	HAL_ADC_Start(&LP_ADC);

}
void LLP_DMA_init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	//TODO function init all interrup (priority)
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	//ADC
	LLP_dma_adc.Instance = DMA2_Stream4;
	LLP_dma_adc.Init.Channel = DMA_CHANNEL_0;
	LLP_dma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	LLP_dma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	LLP_dma_adc.Init.MemInc = DMA_MINC_ENABLE;
	LLP_dma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	LLP_dma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	LLP_dma_adc.Init.Mode = DMA_NORMAL;
	LLP_dma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
	LLP_dma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&LLP_dma_adc) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_LINKDMA(&LP_ADC, DMA_Handle, LLP_dma_adc);// ADC can control DMA

	//SPI TX
	LLP_dma_spi4_tx.Instance = DMA2_Stream1;
	LLP_dma_spi4_tx.Init.Channel = DMA_CHANNEL_4;
	LLP_dma_spi4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	LLP_dma_spi4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	LLP_dma_spi4_tx.Init.MemInc = DMA_MINC_ENABLE;
	LLP_dma_spi4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	LLP_dma_spi4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	LLP_dma_spi4_tx.Init.Mode = DMA_NORMAL;
	LLP_dma_spi4_tx.Init.Priority = DMA_PRIORITY_LOW;
	LLP_dma_spi4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&LLP_dma_spi4_tx) != HAL_OK)
	{
	  Error_Handler();
	}
	__HAL_LINKDMA(&LLP_hspi4, hdmatx, LLP_dma_spi4_tx);// SPI can control DMA

	//SPI RX
	LLP_dma_spi4_rx.Instance = DMA2_Stream0;
	LLP_dma_spi4_rx.Init.Channel = DMA_CHANNEL_4;
	LLP_dma_spi4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	LLP_dma_spi4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	LLP_dma_spi4_rx.Init.MemInc = DMA_MINC_ENABLE;
	LLP_dma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	LLP_dma_spi4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	LLP_dma_spi4_rx.Init.Mode = DMA_NORMAL;
	LLP_dma_spi4_rx.Init.Priority = DMA_PRIORITY_LOW;
	LLP_dma_spi4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&LLP_dma_spi4_rx) != HAL_OK)
	{
	  Error_Handler();
	}
	__HAL_LINKDMA(&LLP_hspi4,hdmarx,LLP_dma_spi4_rx);// SPI can control DMA
}
void LLP_DREQ_mode_interrup(void)
{
	LLP_gpio.Pin = LP_pin_DREQ;
	LLP_gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_RES, &LLP_gpio);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
void LLP_DREQ_mode_pooling(void)
{
	LLP_gpio.Pin = LP_pin_DREQ;
	LLP_gpio.Mode = GPIO_MODE_INPUT;
	LLP_gpio.Pull = GPIO_PULLUP;
	LLP_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LP_port_RES, &LLP_gpio);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}
void LLP_SPI_write(uint16_t* tx_buff , uint16_t size)
{
	HAL_SPI_Transmit(&LLP_hspi4, tx_buff, size, HAL_MAX_DELAY);
}
void LLP_SPI_read(uint16_t* rx_buff , uint16_t size)
{
	HAL_SPI_Receive(&LLP_hspi4, rx_buff, size, HAL_MAX_DELAY);
}
void LLP_SPI_read_write(uint16_t* tx_buff,uint16_t* rx_buff , uint16_t size)
{
	HAL_SPI_TransmitReceive(&LLP_hspi4, tx_buff, rx_buff, size, HAL_MAX_DELAY);
}
void LLP_SPI_CS_SCI_inactive_SDI_active(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_CCS, SET);
}
void LLP_SPI_CS_SCI_active_SDI_inactive(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_CCS, RESET);
}
void LLP_SPI_DCS_inactive(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_DCS, SET);
}
void LLP_SPI_DCS_active(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_DCS, RESET);
}
void LLP_SPI_RES_inactive(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_RES, SET);
}
void LLP_SPI_RES_active(void)
{
	HAL_GPIO_WritePin(LP_port_RES, LP_pin_RES, RESET);
}
void LLP_DREQ_WAIT(void)
{
	//idea time out
	while(HAL_GPIO_ReadPin(LP_port_RES, LP_pin_DREQ)==GPIO_PIN_RESET)
	{}
}

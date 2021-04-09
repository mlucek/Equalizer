#include "ADC_NUCLEO.h"




void send_char(char c)
{
	HAL_UART_Transmit(&uart, (uint8_t*)&c, 1, 1000);
}
int __io_putchar(int ch)
{
	if (ch == '\n')
		send_char('\r');
	send_char(ch);
	return ch;
}
int adc_read(uint32_t channel)
{
	ADC_ChannelConfTypeDef adc_ch;
	adc_ch.Channel = channel;
	adc_ch.Rank = ADC_REGULAR_RANK_1;
	adc_ch.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	HAL_ADC_ConfigChannel(&adc, &adc_ch);

    HAL_ADC_Start(&adc);
	HAL_ADC_PollForConversion(&adc, 1000);
    return HAL_ADC_GetValue(&adc);
}
void UART_Init()
{

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_2;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &gpio);


	uart.Instance = USART2;
	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.StopBits = UART_STOPBITS_1;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OverSampling = UART_OVERSAMPLING_16;
	uart.Init.Mode = UART_MODE_TX_RX;
	HAL_UART_Init(&uart);


}
void ADC_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	GPIO_InitTypeDef gpio;

	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_4 | GPIO_PIN_9;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOB, &gpio);



	RCC_PeriphCLKInitTypeDef adc_clk;
	adc_clk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	adc_clk.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	HAL_RCCEx_PeriphCLKConfig(&adc_clk);

	adc.Instance = ADC1;
	adc.Init.ContinuousConvMode = DISABLE;
	adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	adc.Init.NbrOfConversion = 1;
	adc.Init.DiscontinuousConvMode = DISABLE;
	adc.Init.NbrOfDiscConversion = 1;
	HAL_ADC_Init(&adc);

	HAL_ADCEx_Calibration_Start(&adc);

}
void SPI_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();


	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7; // SCK, MOSI
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);


	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_6; // MISO
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_0; // CS
	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


	 spi.Instance = SPI1;
	 spi.Init.Mode = SPI_MODE_MASTER;
	 spi.Init.NSS = SPI_NSS_SOFT;
	 spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;	// 1MHz
	 spi.Init.Direction = SPI_DIRECTION_2LINES;
	 spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	 spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	 spi.Init.DataSize = SPI_DATASIZE_8BIT;
	 spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	 spi.Init.TIMode = SPI_TIMODE_DISABLE;
	 spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	 spi.Init.CRCPolynomial = 7;
	 HAL_SPI_Init(&spi);

	__HAL_SPI_ENABLE(&spi);
}
void SPI_Send_Bit(uint8_t byte)
{
	HAL_SPI_Transmit(&spi, &byte,1,HAL_MAX_DELAY);
}
void SPI_WRITE_ADC(uint8_t CH0,uint8_t CH1,uint8_t CH4,uint8_t CH8)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	SPI_Send_Bit(CH0);
	SPI_Send_Bit(CH1);
	SPI_Send_Bit(CH4);
	SPI_Send_Bit(CH8);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

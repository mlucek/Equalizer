#include <string.h>
#include "stm32f1xx.h"

UART_HandleTypeDef uart;
ADC_HandleTypeDef adc;
SPI_HandleTypeDef spi;

#define PIN_A_0		ADC1_CH_0
#define PIN_A_1		ADC1_CH_1
#define PIN_A_2		UART2_TX
#define PIN_A_3		UART2_RX
#define PIN_A_4		ADC1_CH4
#define PIN_A_5		SPI1_SCK
#define PIN_A_6		SPI1_MISO
#define PIN_A_7		SPI1_MOSI
#define PIN_B_0		ADC1_CH8
#define PIN_C_0		SPI1_CS


void send_char(char c);
int __io_putchar(int ch);
int adc_read(uint32_t channel);
void UART_Init();
void ADC_Init();
void SPI_init();
void SPI_Send_Bit(uint8_t byte);
void SPI_WRITE_ADC(uint8_t CH0,uint8_t CH1,uint8_t CH4,uint8_t CH8);

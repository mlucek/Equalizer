/*
 * layers_port.h
 *
 * @author  Lucjan Maciej, Zachariasz Monka
 *
 */

//config interrup tim10!

#ifndef LAYERS_PORT_H_
#define LAYERS_PORT_H_

#ifndef includestm32f4xx
	#define includestm32f4xx_h_
	#include "stm32f4xx.h"
#endif //includestm32f4xx_h_



//todo pin and port in LLP
#define LP_pin_LED_BLUE GPIO_PIN_15
#define LP_pin_LED_RED GPIO_PIN_14
#define LP_pin_LED_ORANGE GPIO_PIN_13
#define LP_pin_LED_GREEN GPIO_PIN_12
#define LP_port_LED GPIOD

#define LP_pin_MOSI GPIO_PIN_14
#define LP_pin_MISO GPIO_PIN_13
#define LP_pin_SCK  GPIO_PIN_12
#define LP_port_SPI GPIOE

#define LP_pin_CCS  GPIO_PIN_6
#define LP_pin_DCS  GPIO_PIN_7
#define LP_pin_DREQ  GPIO_PIN_1
#define LP_pin_RES  GPIO_PIN_2
#define LP_port_RES  GPIOD //todo

#define LP_pin_ADC  GPIO_PIN_1
#define LP_port_ADC  GPIOA

#define LP_pin_SPI_ADC_CS  GPIO_PIN_3
#define LP_port_SPI_ADC_CS  GPIOD


//TODO encapsulation LLP
GPIO_InitTypeDef LLP_gpio;
volatile TIM_HandleTypeDef LLP_tim10;
volatile SPI_HandleTypeDef LLP_hspi4;
volatile ADC_HandleTypeDef LP_ADC;
volatile DMA_HandleTypeDef LLP_dma_spi4_tx;
volatile DMA_HandleTypeDef LLP_dma_spi4_rx;
volatile DMA_HandleTypeDef LLP_dma_adc;
volatile TIM_HandleTypeDef tim2; //todo LLP
volatile TIM_OC_InitTypeDef OC; //todo LLP
volatile SPI_HandleTypeDef LLP_hspi5;
volatile DMA_HandleTypeDef LLP_dma_spi5_rx;
volatile DMA_HandleTypeDef LLP_dma_spi5_tx;





typedef enum
{
	LP_LED_OFF = 0U,
	LP_LED_ON = 1U,
	//LP_LED_BLINK_1Hz = 2U, //todo
	//LP_LED_BLINK_5Hz = 3U, //todo
	LP_LED_TOGLE = 4U,
} LP_LED_STATUS;

typedef enum
{
	LP_LED_BLUE = 0U,
	LP_LED_RED = 1U,
	LP_LED_ORANGE = 2U,
	LP_LED_GREEN = 3U,
} LP_LED_COLOR;

typedef enum
{
	LLP_ADC_WAIT = 0U,
	LLP_ADC_READY= 1U,
	LLP_ADC_HALF= 2U,

} LP_ADC_state;

volatile LP_LED_STATUS LLP_led_tab[4];
volatile uint8_t LLP_tim10_cycle;
volatile LP_ADC_state LLP_ADC_tab;
volatile uint8_t LP_ADC_EXTERNAL[4];


void SystemClock_Config(void);
void Error_Handler(void);
void LP_Delay(uint32_t Delay);

volatile void LLP_interrup_EXTI1(void);
volatile void LLP_iunerrup_dma_tx(void);
volatile void LLP_iunerrup_tim10(void);
void LP_init(void);

void LP_LED(LP_LED_COLOR color, LP_LED_STATUS status);
void LP_SPI_low_speed(void);
void LP_SPI_high_speed(void);
uint16_t LP_VS1003_register_read(uint8_t register_adres);
void LP_VS1003_register_write(uint8_t register_adres, uint16_t data);
void LP_VS1003_set_bit(uint8_t register_adres, uint16_t bit);
void LP_VS1003_reset_bit(uint8_t register_adres, uint16_t bit);
void LP_VS1003_WRITE_DATA_pooling(uint16_t* data,uint16_t size);
void LP_VS1003_WRITE_DATA(uint16_t* data,uint16_t size);
void LP_VS1003_WRITE_DATA_wait_for_end(void);
void LP_VS1003_Hardware_reset(void);
void LP_ADC_read(uint16_t* data,uint16_t size);
void LP_ADC_wait_FULL(void);
void LP_ADC_wait_HALF(void);
uint8_t LP_ADC_EXTERNAL_CH_L(void);
uint8_t LP_ADC_EXTERNAL_CH_M(void);
uint8_t LP_ADC_EXTERNAL_CH_H(void);
uint8_t LP_ADC_EXTERNAL_CH_V(void);

void LLP_TIM2_init(void);
void LLP_ADC_init(void);
void LLP_DMA_init(void);
void LLP_DREQ_mode_interrup(void);
void LLP_DREQ_mode_pooling(void);
void LLP_SPI_write(uint16_t* tx_buff , uint16_t size);
void LLP_SPI_read(uint16_t* rx_buff , uint16_t size);
void LLP_SPI_read_write(uint16_t* tx_buff,uint16_t* rx_buff , uint16_t size);
void LLP_SPI_CS_SCI_inactive_SDI_active(void);
void LLP_SPI_CS_SCI_active_SDI_inactive(void);
void LLP_SPI_DCS_inactive(void);
void LLP_SPI_DCS_active(void);
void LLP_SPI_RES_inactive(void);
void LLP_SPI_RES_active(void);
void LLP_DREQ_WAIT(void);


#endif // LAYERS_PORT_H_

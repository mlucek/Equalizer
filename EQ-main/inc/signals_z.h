/*
 * sygnal.h
 *
 * @author  Lucjan Maciej, Zachariasz Monka
 *
 */


#ifndef SIGNALS_Z_H_
#define SIGNALS_Z_H_

#ifndef includestm32f4xx
	#define includestm32f4xx_h_
	#include "stm32f4xx.h"
#endif //includestm32f4xx_h_

uint16_t ssyg_last_sample[8];

void syg_uint_to_int(uint16_t* in_buff , int16_t* out_buff , uint16_t size);
void syg_int_to_uint(int16_t* in_buff , uint16_t* out_buff , uint16_t size);
void syg_init(void);
void syg_decoding(uint16_t* in_buff , uint16_t* out_buff , uint16_t size);
void syg_FIR(int16_t* in_buff , int16_t* out_buff , uint16_t size,int16_t* FIR_table);
void syg_mix2(uint16_t* inA_buff ,uint8_t scaleA , uint16_t* inB_buff, uint8_t scaleB, uint16_t* out_buff , uint16_t size);

#endif // SIGNALS_Z_H_

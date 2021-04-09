/*
 * sygnal.h
 *
 * @author  Lucjan Maciej, Zachariasz Monka
 *
 */

#include "signals_z.h"

void syg_uint_to_int(uint16_t* in_buff , int16_t* out_buff , uint16_t size)
{
	int16_t buff = 0;
	for(uint16_t i=0; i < size; i++)
	{
		buff = in_buff[i];
		out_buff[i] = buff - 32768;
	}
}
void syg_int_to_uint(int16_t* in_buff , uint16_t* out_buff , uint16_t size)
{
	//com
	int16_t buff = 0;
	for(uint16_t i=0; i < size; i++)
	{
		buff = in_buff[i] + 32768;
		out_buff[i] = buff;
	}
}
void syg_init(void)
{
	ssyg_last_sample[0] = 0;
	ssyg_last_sample[1] = 0;
	ssyg_last_sample[2] = 0;
	ssyg_last_sample[3] = 0;
	ssyg_last_sample[4] = 0;
	ssyg_last_sample[5] = 0;
	ssyg_last_sample[6] = 0;
	ssyg_last_sample[7] = 0;
}
void syg_decoding(uint16_t* in_buff , uint16_t* out_buff , uint16_t size)
{
	for(uint16_t i=0; i < size; i++)
	{
		out_buff[i] = in_buff[i] * 16;	//alias to left (change 12b signal to 16b)

		if(out_buff[i] > 32768 ) 		//code to PCM
			out_buff[i] = out_buff[i] - 32768;
		else
			out_buff[i] = out_buff[i] + 32768;

		out_buff[i] = out_buff[i]<<8 | out_buff[i]>>8;  //little endian to big endian
	}
}
void syg_FIR(int16_t* in_buff , int16_t* out_buff , uint16_t size,int16_t* FIR_table)
{
	const uint16_t FIR_length = 8;

	//first eight sample
	for(uint16_t i=0; i < FIR_length; i++)
	{
		uint16_t temp = 0; //temporary variable used for summation
		for(uint16_t f=0; f < FIR_length; f++)
		{
			if(i > f)//checks if the data is in this table
				temp += FIR_table[f] * in_buff[i - f]/(FIR_length*10);
			else//if the data is in old samples
				temp += FIR_table[f] * ssyg_last_sample[FIR_length + i - f]/(FIR_length*10);
		}
		out_buff[i] = temp;
	}
	//samples between the first eight and last eight
	for(uint16_t i=FIR_length; i < size - FIR_length; i++)
	{
		uint16_t temp = 0; //temporary variable used for summation
		for(uint16_t f=0; f < FIR_length; f++)
		{
			temp += FIR_table[f] * in_buff[i - f]/(FIR_length*10);
		}
		out_buff[i] = temp;
	}
	//last eight sample
	for(uint16_t i= size -FIR_length; i < size; i++)
	{
		ssyg_last_sample[i - size + 8 ] = in_buff[i]; //save last samples

		uint16_t temp = 0; //temporary variable used for summation
		for(uint16_t f=0; f < FIR_length; f++)
		{
			temp += FIR_table[f] * in_buff[i - f]/(FIR_length*10);
		}
		out_buff[i] = temp;
	}
}
void syg_mix2(uint16_t* inA_buff ,uint8_t scaleA , uint16_t* inB_buff, uint8_t scaleB, uint16_t* out_buff , uint16_t size)
{

	uint16_t scaleA_norm;
	uint16_t scaleB_norm;
	scaleA_norm = (scaleA / 16);
	scaleB_norm = (scaleB / 16);
	for(uint16_t i=0; i < size; i++)
	{
		out_buff[i] = (inA_buff[i] * scaleA_norm)/32 + (inB_buff[i] * scaleB_norm)/32;
	}
}

/*
 * VS1003b.c
 *
 * @author  Lucjan Maciej, Zachariasz Monka
 *
 */

#include "VS1003b.h"


uint16_t clean[2048] = {0}; /// clean Buffer
void VS1003b_Init()
{
	LP_VS1003_Hardware_reset();
	LP_Delay(5);
	LP_VS1003_set_bit(SCI_MODE,SM_SDISHARE);  //turn off xDCS turn on xCS to both SCI and SDI
	LP_Delay(5);
}
void VS1003b_Set_Freq_Mult(uint16_t Mult)
{
	//basic clock Frequency: 12.288 Mhz but to be sure write SC_Mult_....|SC_SC_12_288Mhz
	//Multiplication1 up to 4.5 with 0.5 step
	// use prefix SC_MULT_ to chose
	LP_VS1003_set_bit(SCI_CLOCKF,Mult);
}
void VS1003b_Record_ADPCM(uint8_t* output, uint16_t output_size)
{
	//size of the recording should be min 100x256 bytes block because smaller recording dont work correct
	uint16_t w = 0, idx =0;
	uint8_t stop =output_size/256;
	uint16_t increment=0;
	idx=0;

	while (stop!=0) {
		do {
			w = LP_VS1003_register_read(SCI_HDAT1);
		} while (w < 256 || w >= 896);
		while (idx < 256) {

			w=LP_VS1003_register_read(SCI_HDAT0);

			output[increment + idx++] = w>>8;
			output[increment + idx++] = w & 0xFF;


		}
		stop -- ;
		increment +=256;
		idx =0;
	}
}
/*
void VS1003b_Record_PCM(uint16_t* output, uint16_t output_size)
{

	//// if you use sizeof function you should siezof(output)/2 because sizeof() returns 8 bits size ///
	/// for use this function you should downland "VS1003B PCM Encoder" from http://www.vlsi.fi/en/support/software/vs10xxapplications.html
	/// but 23.01.2021 available is v0.1 version and its dont work correct
	uint16_t w = 0, idx =0,test=0;
	uint16_t stop =output_size/256;
	uint16_t increment=0;
	idx=0;

	while (stop!=0) {
		do {
			w = LP_VS1003_register_read(SCI_HDAT1);
		} while (w < 256 || w >= 896);
		while (idx < 256) {
			w=LP_VS1003_register_read(SCI_HDAT0);
			output[increment + idx++] = w;

		}
		stop -- ;
		increment +=256;
		idx =0;
	}
}
*/
void VS1003b_Record_with_header_ADPCM(uint8_t* header,uint16_t header_size,uint8_t* output, uint16_t output_size)
{
	//size of the recording should be min 100x256 bytes block because smaller recording dont work correct
	uint16_t w = 0, idx =0;

	for (idx=0; idx < header_size; idx++) {
		output[idx] = header[idx];
	}

	uint8_t stop =(output_size-header_size)/256;
	uint16_t increment=header_size;
	idx=0;

	while (stop!=0) {
		do {
			w = LP_VS1003_register_read(SCI_HDAT1);
		} while (w < 256 || w >= 896);
		while (idx < 256) {

			w=LP_VS1003_register_read(SCI_HDAT0);

			output[increment + idx++] = w>>8;
			output[increment + idx++] = w & 0xFF;

		}
		stop -- ;
		increment +=256;
		idx =0;
	}
}
/*
void VS1003b_Record_with_header_PCM(uint16_t* header,uint16_t header_size,uint16_t* output, uint16_t output_size)
{
	//// if you use sizeof function you should siezof(output)/2 because sizeof() returns 8 bits size ///
	/// for use this function you should downland "VS1003B PCM Encoder" from http://www.vlsi.fi/en/support/software/vs10xxapplications.html
	/// but 23.01.2021 available is v0.1 version and its dont work correct
	uint16_t w = 0, idx =0;

	for (idx=0; idx < header_size; idx++) {
		output[idx] = header[idx];
	}

	uint8_t stop =(output_size-header_size)/256;
	uint16_t increment=header_size;
	idx=0;

	while (stop!=0) {
		do {
			w = LP_VS1003_register_read(SCI_HDAT1);
		} while (w < 256 || w >= 896);
		while (idx < 256) {

			w=LP_VS1003_register_read(SCI_HDAT0);

			output[increment + idx++] = w>>8;



		}
		stop -- ;
		increment +=256;
		idx =0;

	}
}
*/
void VS1003b_8bit_to_16Bit(uint8_t* input,uint16_t input_size,uint16_t* output, uint16_t output_size)
//// if you use sizeof function you should write sizeof(input) and siezof(output)/2 because sizeof() returns 8 bits size ///
///  this function allows to send 2 bytes block
{
if(input_size==2*output_size)
	{
	uint16_t idx;
	uint16_t idx2=0;
	for (idx=0;idx<output_size;idx++){
		uint16_t z1,z2;
		z1=input[idx2++];
		z2=input[idx2++];
		output[idx]=(z1<<8)|(z2);
		}
	}
}
void VS1003b_Play_48kHz_Init()
{
	///// This function allows to send PCM data with mono channel in a continuous stream
	///// The sample Rate should be 48 kHz
	uint16_t headerPCM [] = {
	0x5249, 0x4646,     //	----RIFF
	0x0000, 0x0000,		//	----data size  for stream 0x0000, 0x0000
	0x5741, 0x5645, 	//	----Wave
	0x666D, 0x7420,		//	----fmt
	0x1000, 0x0000,		//	----16
	0x0100, 			//	----pcm
	0x0100,				//	----channel 1
	0x80BB, 0x0000,		//	----sampleRate	0x401f, 0x0000<--- 8khz	////	0x803e, 0x0000 <---16khz	/////0x44AC, 0x0000 <---44,1khz ////  0x80BB, 0x0000  <-----48kHz
	0x0077, 0x0100,		//	----bytesPersecond    ((sampleRate*channel *16)/8)  ///    0x0077, 0x0100, <-------48khz(mono)
	0x0200,				//	----bytesPerSample              ///// 0x0200     <-------48khz(mono)
	0x1000,				//	----bit per sample 16(10h)
	0x6461, 0x7461,		//	----Data
	0x0000, 0x0000		//	----data size    for stream 0x0000, 0x0000
	};

	VS1003b_Set_Freq_Mult(SC_MULT_4_XTALI_MASK|SC_12_288Mhz);  //// this function allow send data with 7 Mhz (49Mhz/7)
	LP_Delay(5);
	LP_SPI_high_speed();
	LP_VS1003_WRITE_DATA_pooling(clean,sizeof(clean)/2);
	LP_VS1003_WRITE_DATA_pooling(headerPCM,sizeof(headerPCM)/2);
	LP_VS1003_WRITE_DATA_pooling(headerPCM,sizeof(headerPCM)/2);

}
void VS1003b_test_sine()		//Sin test 5168Hz  ///5s
{
	uint16_t input[] = {0x53EF,0x6E7E,0x00,0x00};  	//turn on sequence
	uint16_t input2[] = {0x4578,0x6974,0x00,0x00};	//turn off sequence
	LP_init();
	LP_VS1003_Hardware_reset();									//Reset
	LP_VS1003_set_bit(SCI_MODE,SM_TESTS);						//turn on tests option
	LP_VS1003_set_bit(SCI_MODE,SM_SDISHARE);					//turn off xDCS turn on xCS to both SCI and SDI
	LP_VS1003_WRITE_DATA(input,sizeof(input));
	LP_VS1003_WRITE_DATA_wait_for_end();
	LP_Delay(5000);
	LP_VS1003_WRITE_DATA(input2,sizeof(input2));
	LP_VS1003_WRITE_DATA_wait_for_end();
}
void VS1003b_Finish()
{
	LP_VS1003_set_bit(SCI_MODE,SM_OUTOFWAV);

	LP_Delay(2000);

	LP_VS1003_set_bit(SCI_MODE,SM_RESET);
	LP_Delay(5);
}
void VS1003b_Play(uint16_t* buff,uint16_t buff_size)
{
	LP_VS1003_WRITE_DATA(buff,buff_size);
}
void VS1003b_END_Play()
{
	LP_VS1003_WRITE_DATA_wait_for_end();
}
void VS1003b_set_BASS_FREQ(uint8_t bassfreq)
{
	//bass band 0- 0ff
	//2-15 *10hz steps
	LP_VS1003_set_bit(SCI_BASS,bassfreq);
}
void VS1003b_set_BASS_Enhancemeeps(uint8_t bassenhancemeeps)
{
	//set Bass Enhancement in 1 dB steps (0..15, 0 = off)
	LP_VS1003_set_bit(SCI_BASS,bassenhancemeeps<<4);
}
void VS1003b_set_TRABLE_FREQ(uint8_t trablefreq)
{
	//Treble Lower limit frequency in 1000 Hz steps (0..15)
	LP_VS1003_set_bit(SCI_BASS,trablefreq<<8);
}
void VS1003b_set_TRABLE_CONTRO(uint8_t trablecontrol)
{
	//Treble Control in 1.5 dB steps (-8..7, 0 = off)
	LP_VS1003_set_bit(SCI_BASS,trablecontrol<<12);
}
void VS1003b_set_VOL_RIGHT(uint8_t vol)
{
	LP_VS1003_set_bit(SCI_VOL,vol);
}
void VS1003b_set_VOL_LEFT(uint8_t vol)
{
	LP_VS1003_set_bit(SCI_VOL,vol<<8);
}
void VS1003b_set_VOL(uint8_t vol)
{
	VS1003b_set_VOL_LEFT(vol);
	VS1003b_set_VOL_RIGHT(vol);
}
void VS1003b_set_DIFF()
{
	LP_VS1003_set_bit(SCI_MODE,SM_DIFF);
}

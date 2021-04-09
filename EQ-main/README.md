# TM2
Project for studies for the subject of microprocessor technology 2.

Digital equalizer based on STM32F411 Using DMA to the I2S transmitter and receiver and a potentiometer connected to the ADC to adjust the band.

links to the elements used below:
https://botland.com.pl/pl/stm32-discovery/3563-stm32f411e-disco-discovery-stm32f411ediscovery.html
https://botland.com.pl/pl/odtwarzacze-mp3-wav-oog-midi/4488-vs1003b-odtwarzacz-mp3-modul-dzwiekowy-z-mikrofonem-waveshare-4038.html


Basic assumptions:  
-low-pass filter  
-high-pass filter  
-band pass filter  
-audio transmitted via mini jack( outpuut and input)  
-audio processing with minimal delay( DMA)  
-I2S to comunication between STM32F411 and sound module  

#include "spi.h"
#include "24cxx.h"



uint8_t SPI_ADS_SendData(uint8_t Data)                             //??????????
{
	uint8_t data=0;
	
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); //???????
		SPI_I2S_SendData(SPI2, Data);								   //??SPI2??????
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);//????????
		data = SPI_I2S_ReceiveData(SPI2);			
	
	return data;												   //????SPI2?????
}

#ifndef __SPI_H
#define __SPI_H
#include "stm32f4xx.h"
							  
uint8_t SPI_ADS_SendData(uint8_t Data);

//			#define DISABLE_BMI055A_SPI	     GPIO_SetBits(GPIOA, GPIO_Pin_15);
//			#define ENABLE_BMI055A_SPI       GPIO_ResetBits(GPIOA, GPIO_Pin_15);

//			#define DISABLE_BMI055G_SPI	     GPIO_SetBits(GPIOA, GPIO_Pin_4);
//			#define ENABLE_BMI055G_SPI       GPIO_ResetBits(GPIOA, GPIO_Pin_4);

			#define SETBMI160 GPIO_SetBits(GPIOA, GPIO_Pin_4);
			#define RESETBMI160 GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#endif


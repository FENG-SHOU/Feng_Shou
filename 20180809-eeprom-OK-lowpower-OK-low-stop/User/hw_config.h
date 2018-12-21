#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "24cxx.h"

extern uint32_t  USART2_BodRate;

void Set_System(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void SPI_Configuration(void);
void I2C_Configuration(void);	  // 配置I2C口
void USART_Configuration(void);
void USART_Configuration6(void);
void TIM3_Configuration(void);
void EXTIX_Init(void);	//外部中断
void NVIC_Configuration(void);
void NVIC_ConfigurationDis(void);
void USART_DMA_TX_Configuration(uint8_t *DMA_TTX_Data);

void SendDriver(void);
#endif

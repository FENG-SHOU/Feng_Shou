#ifndef   __ENCRYPTID_H
#define   __ENCRYPTID_H

#include  "stdint.h"
#include  "stm32f4xx.h"

#define ID_BaseAddress0   0X1FFF7A10
#define ID_BaseAddress1	  0X1FFF7A14
#define ID_BaseAddress2	  0X1FFF7A18

//----------------------------------------------
static uint16_t   CPU_ID[6];
static uint32_t   CPU_Get_ID[3];		                 //通过寻址获取的芯片的ID

//----------------------------------------------
extern uint8_t    CPU_ID_Data[25];



void Check_Lock_Code(void);  

#endif


#ifndef __EEPROM_H
#define __EEPROM_H
#include "myiic.h"   


					  
u8 EEPROM_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节
void EEPROM_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节
void EEPROM_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 EEPROM_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void EEPROM_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void EEPROM_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据
void EEPROM_ReadID(void);

u8 EEPROM_Check(void);  //检查器件
void EEPROM_Init(void); //初始化IIC
#endif
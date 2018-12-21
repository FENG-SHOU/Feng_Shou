#ifndef __24CXX_H
#define __24CXX_H
#include "myiic.h"   
	

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767  
#define AT24C512  65535	

#define EE_TYPE AT24C512

#define USE_I2C1   1     //1, USE_I2C1  //new 
                         //0, USE_I2C2  //old
												 
#define USE_SPI    1     //1, SPI1  //new 
                         //0, SPI2  //old				

#define SOFT_I2C   1     //1, Soft
                         //0, Hard
												 
#if  USE_I2C1 
   #define I2C_Num  I2C1
#else
   #define I2C_Num  I2C2
#endif


					  
u8 AT24CXX_ReadOneByte(u8 ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 AT24CXX_ReadLenByte(u8 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//从指定地址开始读出指定长度的数据


void _alpu_delay_ms(unsigned int i);
unsigned char _alpu_rand(void);  
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte);
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte);


u8 AT24CXX_Check(void);  //检查器件
void AT24CXX_Init(void); //初始化IIC
#endif

















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


					  
u8 AT24CXX_ReadOneByte(u8 ReadAddr);							//ָ����ַ��ȡһ���ֽ�
void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite);		//ָ����ַд��һ���ֽ�
void AT24CXX_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 AT24CXX_ReadLenByte(u8 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void AT24CXX_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void AT24CXX_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����


void _alpu_delay_ms(unsigned int i);
unsigned char _alpu_rand(void);  
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte);
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte);


u8 AT24CXX_Check(void);  //�������
void AT24CXX_Init(void); //��ʼ��IIC
#endif

















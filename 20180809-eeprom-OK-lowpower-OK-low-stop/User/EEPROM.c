#include "EEPROM.h" 
#include "myiic.h"
#include "delay.h" 				 
#include <stdlib.h>
#include "stm32f4xx_i2c.h"


//初始化IIC接口
void EEPROM_Init(void)
{
	IIC_Init();
}

//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 EEPROM_ReadOneByte(u16 ReadAddr)
{				  
	  u16 temp=0;		  	    																 
    IIC_Start();  
		IIC_Send_Byte(0XA0);	   //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr/256);//发送高地址
	  IIC_Wait_Ack(); 
    IIC_Send_Byte(ReadAddr%256);   //发送低地址
	  IIC_Wait_Ack();	    
	  IIC_Start();  	 	   
	  IIC_Send_Byte(0XA1);           //进入接收模式			   
	  IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	  return temp;
}


//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void EEPROM_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{		
	  IIC_Start();  
		IIC_Send_Byte(0xa0);	    //发送写命令
		IIC_Wait_Ack();	
		IIC_Send_Byte(WriteAddr/256);//发送高地址	  
	  IIC_Wait_Ack();	   
		IIC_Send_Byte(WriteAddr%256);   //发送低地址
		IIC_Wait_Ack();	 										  		   
		IIC_Send_Byte(DataToWrite);     //发送字节							   
		IIC_Wait_Ack();	   	   
		IIC_Stop();//产生一个停止条件 
		delay_ms(10);	 
}

//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void EEPROM_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u16 t;
	for(t=0;t<Len;t++)
	{
		EEPROM_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 EEPROM_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u16 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=EEPROM_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

//返回1:检测失败
//返回0:检测成功
u8 EEPROM_Check(void)
{
	u16 temp;
	temp=EEPROM_ReadOneByte(65535);
	if(temp==0X55)
	return 0;		   
	else//排除第一次初始化的情况
	{
		EEPROM_WriteOneByte(65535,0X55);
	  temp=EEPROM_ReadOneByte(65535);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//ReadAddr :开始读出的地址0-65535
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void EEPROM_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=EEPROM_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

//WriteAddr :开始写入的地址 0-65535
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void EEPROM_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		EEPROM_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}



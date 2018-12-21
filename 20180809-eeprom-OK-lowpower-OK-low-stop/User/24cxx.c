#include "24cxx.h" 
#include "myiic.h"
#include "delay.h" 				 
#include <stdlib.h>
#include "stm32f4xx_i2c.h"

u8 check1 = 0x11;
extern int16_t  Gyr_Z;

u8 adress = 0XA0;   //0x7A 是机密芯片  ，OXA0是EEPROM

//初始化IIC接口
void AT24CXX_Init(void)
{
	IIC_Init();//IIC初始化
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 AT24CXX_ReadOneByte(u8 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(adress);	   //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8);//发送高地址	    
	}else IIC_Send_Byte(0X7A+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	   
	while(IIC_Wait_Ack()){};
  IIC_Send_Byte(ReadAddr%256);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(adress+1);           //进入接收模式			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
  IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(adress);	    //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址	  
	}else IIC_Send_Byte(adress+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	while(IIC_Wait_Ack()){};	   
  IIC_Send_Byte(WriteAddr%256);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	delay_ms(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u8 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp = 0x22;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX	
  
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);
	    check1 = temp;
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}



//------------------------------------------------------------
//                
//-------------------------------------------------------------

void _alpu_delay_ms(unsigned int i)
{
	delay_ms(i);
}

/******* 产生一个字节的随机数**************/
unsigned char _alpu_rand(void)  // Need for ALPUC lib.
{ 
    static unsigned long seed; // 2byte, must be a static variable
    
	 srand(Gyr_Z);

    seed = seed + rand(); // rand(); <------------------ add time value
    seed = seed * 1103515245 + 12345;

    return (seed/65536) % 32768;

}



void II2C_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
  IIC_Start();  
	IIC_Send_Byte(adress);	                  //发送写命令
	IIC_Wait_Ack();    
  IIC_Send_Byte(WriteAddr);               //发送低地址
	IIC_Wait_Ack(); 	 										  		  							  		   
	IIC_Send_Byte(DataToWrite);             //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();                             //产生一个停止条件 
	delay_ms(10);	 
}


u8 II2C_ReadOneByte(u8 ReadAddr)
{				  
	u8 temp=0;		
	
  IIC_Start();  
	IIC_Send_Byte(adress);	               //发送写命令
	IIC_Wait_Ack();  
  IIC_Send_Byte(ReadAddr);             //发送低地址
	IIC_Wait_Ack();  									 
		
	IIC_Start();  	 	   
	IIC_Send_Byte(adress+1);                 //进入接收模式			   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();                          //产生一个停止条件	    
	
	return temp;
}





#if  SOFT_I2C            //选择软件I2C



unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{
	
	IIC_Start();  
	IIC_Send_Byte(device_addr);	                  //发送写命令
	IIC_Wait_Ack();    
  IIC_Send_Byte(sub_addr);               //发送低地址
	IIC_Wait_Ack(); 	 
	
	while(Byte--)
	{
		IIC_Send_Byte(*buff);             //发送字节							   
	  IIC_Wait_Ack();  
		
		buff++;
	}
	
	IIC_Stop();                             //产生一个停止条件 
	delay_ms(10);	 
	
	return 0;
}


unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{	
	
  IIC_Start();  
	IIC_Send_Byte(device_addr);	               //发送写命令
	IIC_Wait_Ack();  
  IIC_Send_Byte(sub_addr);             //发送低地址
	IIC_Wait_Ack();  
	IIC_Start(); 
	IIC_Send_Byte(device_addr+1);             //发送低地址
	IIC_Wait_Ack();  
	
	while(Byte)
	{
		*buff++=IIC_Read_Byte(0);	
		Byte--;
	}
	
	IIC_Stop();
	
	return 0;
}

#else
	
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{
	u8 i;
	
  Hard_IIC_WriteNByte(I2C1,device_addr,sub_addr,Byte,buff,&i);
	
	return 0;
}




//
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{
	u8 i;
	Hard_IIC_PageRead(I2C1,device_addr,sub_addr,Byte,buff,&i);
   
	return 0;

}
#endif


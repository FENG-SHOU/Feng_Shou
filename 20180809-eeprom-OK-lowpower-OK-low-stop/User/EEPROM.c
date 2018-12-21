#include "EEPROM.h" 
#include "myiic.h"
#include "delay.h" 				 
#include <stdlib.h>
#include "stm32f4xx_i2c.h"


//��ʼ��IIC�ӿ�
void EEPROM_Init(void)
{
	IIC_Init();
}

//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 EEPROM_ReadOneByte(u16 ReadAddr)
{				  
	  u16 temp=0;		  	    																 
    IIC_Start();  
		IIC_Send_Byte(0XA0);	   //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr/256);//���͸ߵ�ַ
	  IIC_Wait_Ack(); 
    IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
	  IIC_Wait_Ack();	    
	  IIC_Start();  	 	   
	  IIC_Send_Byte(0XA1);           //�������ģʽ			   
	  IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	    
	  return temp;
}


//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void EEPROM_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{		
	  IIC_Start();  
		IIC_Send_Byte(0xa0);	    //����д����
		IIC_Wait_Ack();	
		IIC_Send_Byte(WriteAddr/256);//���͸ߵ�ַ	  
	  IIC_Wait_Ack();	   
		IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
		IIC_Wait_Ack();	 										  		   
		IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
		IIC_Wait_Ack();	   	   
		IIC_Stop();//����һ��ֹͣ���� 
		delay_ms(10);	 
}

//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void EEPROM_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u16 t;
	for(t=0;t<Len;t++)
	{
		EEPROM_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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

//����1:���ʧ��
//����0:���ɹ�
u8 EEPROM_Check(void)
{
	u16 temp;
	temp=EEPROM_ReadOneByte(65535);
	if(temp==0X55)
	return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		EEPROM_WriteOneByte(65535,0X55);
	  temp=EEPROM_ReadOneByte(65535);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//ReadAddr :��ʼ�����ĵ�ַ0-65535
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void EEPROM_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=EEPROM_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

//WriteAddr :��ʼд��ĵ�ַ 0-65535
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void EEPROM_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		EEPROM_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}



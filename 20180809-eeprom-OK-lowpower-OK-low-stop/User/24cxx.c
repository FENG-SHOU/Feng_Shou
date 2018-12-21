#include "24cxx.h" 
#include "myiic.h"
#include "delay.h" 				 
#include <stdlib.h>
#include "stm32f4xx_i2c.h"

u8 check1 = 0x11;
extern int16_t  Gyr_Z;

u8 adress = 0XA0;   //0x7A �ǻ���оƬ  ��OXA0��EEPROM

//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	IIC_Init();//IIC��ʼ��
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 AT24CXX_ReadOneByte(u8 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(adress);	   //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ	    
	}else IIC_Send_Byte(0X7A+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
	while(IIC_Wait_Ack()){};
  IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(adress+1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
  IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(adress);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ	  
	}else IIC_Send_Byte(adress+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	while(IIC_Wait_Ack()){};	   
  IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp = 0x22;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX	
  
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);
	    check1 = temp;
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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

/******* ����һ���ֽڵ������**************/
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
	IIC_Send_Byte(adress);	                  //����д����
	IIC_Wait_Ack();    
  IIC_Send_Byte(WriteAddr);               //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		  							  		   
	IIC_Send_Byte(DataToWrite);             //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();                             //����һ��ֹͣ���� 
	delay_ms(10);	 
}


u8 II2C_ReadOneByte(u8 ReadAddr)
{				  
	u8 temp=0;		
	
  IIC_Start();  
	IIC_Send_Byte(adress);	               //����д����
	IIC_Wait_Ack();  
  IIC_Send_Byte(ReadAddr);             //���͵͵�ַ
	IIC_Wait_Ack();  									 
		
	IIC_Start();  	 	   
	IIC_Send_Byte(adress+1);                 //�������ģʽ			   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();                          //����һ��ֹͣ����	    
	
	return temp;
}





#if  SOFT_I2C            //ѡ�����I2C



unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{
	
	IIC_Start();  
	IIC_Send_Byte(device_addr);	                  //����д����
	IIC_Wait_Ack();    
  IIC_Send_Byte(sub_addr);               //���͵͵�ַ
	IIC_Wait_Ack(); 	 
	
	while(Byte--)
	{
		IIC_Send_Byte(*buff);             //�����ֽ�							   
	  IIC_Wait_Ack();  
		
		buff++;
	}
	
	IIC_Stop();                             //����һ��ֹͣ���� 
	delay_ms(10);	 
	
	return 0;
}


unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int Byte)
{	
	
  IIC_Start();  
	IIC_Send_Byte(device_addr);	               //����д����
	IIC_Wait_Ack();  
  IIC_Send_Byte(sub_addr);             //���͵͵�ַ
	IIC_Wait_Ack();  
	IIC_Start(); 
	IIC_Send_Byte(device_addr+1);             //���͵͵�ַ
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


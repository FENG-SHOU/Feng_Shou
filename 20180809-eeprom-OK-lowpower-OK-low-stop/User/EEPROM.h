#ifndef __EEPROM_H
#define __EEPROM_H
#include "myiic.h"   


					  
u8 EEPROM_ReadOneByte(u16 ReadAddr);							//ָ����ַ��ȡһ���ֽ�
void EEPROM_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//ָ����ַд��һ���ֽ�
void EEPROM_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 EEPROM_ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void EEPROM_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void EEPROM_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����
void EEPROM_ReadID(void);

u8 EEPROM_Check(void);  //�������
void EEPROM_Init(void); //��ʼ��IIC
#endif
#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"   
#include "datatypes.h"

//FLASH��ʼ��ַ

#define UpData_Buff_Size  2
#define MAgnle_Buff_Size 36

#define Gyr_Buff_Size     11
#define Acc_Buff_Size     42

#define Gyr_Adress        20
#define Acc_Adress        40

#define Data_Buff_Size    23

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  



extern  uint8_t  Flash_Rdata[150];	
extern  uint8_t  Flash_Wdata[150];

extern uint8_t   NorthSouthSelect;
extern uint8_t   WestEastSelect;

extern uint8_t   NorthSouthBack;
extern uint8_t   WestEastBack;
//---------------------------------------------
extern uint8_t  Flash_Wing_Flag;     //Flashд���־λ
extern uint8_t  Flash_Wirte_Flag; 

extern uint8_t  Data_Flash_W_Flag;   //��־λ----����ϵͳ����д��
extern uint8_t  Update_Flash_W_Flag;
extern uint8_t  MAngle_Flash_W_Flag;

extern uint8_t  Update_One;
extern uint8_t  Update_Two;


extern uint16_t  SVersionH;           // ����汾�� ���
extern uint16_t  SVersionL;           // ����汾�� �·�
extern uint16_t  HVersionH;           // Ӳ���汾�� ���
extern uint16_t  HVersionL;           // Ӳ���汾�� �·�
extern uint16_t  HVersionS;           // Ӳ���汾�� ���

void SoftReset(void);

void Para_Flash_Save(void);
void Para_Flash_INI(void);
void Para_Flash_Decode(void);
void Para_Data_Protection(void);

void Update_Flash_INI(void);
void Update_Flash_Save(void);


void Key_Flash_Decode(void);
void Key_Flash_Save(void);

void MisAngle_Flash_Decode(void);
void MisAngle_Flash_Save(void);
	
//void Gyr_Data_Protection(void);
//void Gyr_Flash_Decode(void);

//void Acc_Data_Protection(void);
//void Acc_Flash_Decode(void);

void Data_Flash_Save(void);
void Data_Flash_Decode(void);

double Buftodouble(uint8_t *Buf,uint8_t counter);


u32 STMFLASH_ReadWord(u32 faddr);		  	//������  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
						   
#endif


















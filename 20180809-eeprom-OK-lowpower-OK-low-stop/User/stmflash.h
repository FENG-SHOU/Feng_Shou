#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"   
#include "datatypes.h"

//FLASH起始地址

#define UpData_Buff_Size  2
#define MAgnle_Buff_Size 36

#define Gyr_Buff_Size     11
#define Acc_Buff_Size     42

#define Gyr_Adress        20
#define Acc_Adress        40

#define Data_Buff_Size    23

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  



extern  uint8_t  Flash_Rdata[150];	
extern  uint8_t  Flash_Wdata[150];

extern uint8_t   NorthSouthSelect;
extern uint8_t   WestEastSelect;

extern uint8_t   NorthSouthBack;
extern uint8_t   WestEastBack;
//---------------------------------------------
extern uint8_t  Flash_Wing_Flag;     //Flash写入标志位
extern uint8_t  Flash_Wirte_Flag; 

extern uint8_t  Data_Flash_W_Flag;   //标志位----启动系统参数写入
extern uint8_t  Update_Flash_W_Flag;
extern uint8_t  MAngle_Flash_W_Flag;

extern uint8_t  Update_One;
extern uint8_t  Update_Two;


extern uint16_t  SVersionH;           // 软件版本号 年份
extern uint16_t  SVersionL;           // 软件版本号 月份
extern uint16_t  HVersionH;           // 硬件版本号 年份
extern uint16_t  HVersionL;           // 硬件版本号 月份
extern uint16_t  HVersionS;           // 硬件版本号 序号

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


u32 STMFLASH_ReadWord(u32 faddr);		  	//读出字  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		//从指定地址开始读出指定长度的数据
						   
#endif


















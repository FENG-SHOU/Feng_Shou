#ifndef  __SENDGPS_H
#define  __SENDGPS_H

#include "stdint.h"




static uint8_t	 GGA_TX_Max;        //每次发送数据的个数
static uint8_t	 RMC_TX_Max;        //每次发送数据的个数
static uint8_t	 VTG_TX_Max;        //每次发送数据的个数
static uint8_t	 ATT_TX_Max;        //每次发送数据的个数
static uint8_t	 GSV_TX_Max;        //每次发送数据的个数
static uint8_t	 ZDA_TX_Max;        //每次发送数据的个数

static uint16_t	 GGA_UP_Num;        //每次发送数据的个数
static uint16_t	 ATT_UP_Num;        //每次发送数据的个数
static uint16_t	 GIRMC_UP_Num;        //每次发送数据的个数
static uint16_t	 GSV_UP_Num;        //每次发送数据的个数
static uint16_t	 ZDA_UP_Num;        //每次发送数据的个数


//-------------------------------------------------------------

extern uint8_t   GGA_Data[20];   
extern uint8_t   GGB_Data[8];        
extern uint8_t   GGS_Data[15];  

extern uint8_t   NorthSouthSelect;
extern uint8_t   WestEastSelect;

extern uint8_t   NorthSouthBack;
extern uint8_t   WestEastBack;


extern uint8_t  IMU_Send_OK;   //IMU数据发送完毕
extern uint8_t  GPS_Send_OK;   //GPS原始数据发送完毕
extern uint8_t  GNSS_Send_OK;  //组合导航数据发送完毕
extern uint8_t  GSA_Send_OK;   //GSA原始数据发送完毕
extern uint8_t  ATT_Send_OK;   //ATT原始数据发送完毕

extern uint8_t  GNSS_Send_OK_Flag;


extern uint8_t	PackDebug_Flag;
extern uint8_t  PackDebug_Time;
extern uint8_t  DMA_Send_Kind; 

extern uint16_t DMA_Tx_Num;




void  GetGPRMC(void);
void  GetGPVTG(void);
void  GetLGPGGA(void);
void  GetAngle(void);
void  Send_GnssData(void);

void ProcessLatToA(double GGAIData);
void ProcessLatToA(double GGAIData);
void ProcessITOA(uint8_t GGAIData);
void ProcessSSFTOA(double GGAIData,uint8_t bit);
void ProcessCheckToA(uint8_t CheckNum);
uint8_t ProcessCheckResult(uint16_t UpDum,uint16_t DNum);
uint8_t ProcessSFTOA(float GGAIData,uint8_t bit);
void Pack_GnssData(void);
void Send_GSAData(void);
void  Pack_Send_Debug(void);


#endif


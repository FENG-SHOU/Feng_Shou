#ifndef __GETIMU_H
#define __GETIMU_H

#include "commdata.h"
#include "GetGyr.h"
#include "GetAcc.h"
#include "GetGPS.h"
#include "GetNass.h"
#include "SendGPS.h"
#include "hw_config.h"
#include "DataProc.h"



extern int16_t   IMU_SData[51][8];  //���ڴ洢����.....
extern uint16_t  DAT_SData[21][4];  //

extern uint8_t   m_IMUData[300];       //
extern uint8_t   m_IMUDataB[300];      //

extern uint8_t   IMU_Send_Flag;

extern uint8_t   IMUTC_Data[20];    //UTCʱ��....
extern uint8_t   IMU_DATAT[60];      //���ڴ��IMU����



//----------------------------------------------------


extern uint8_t   FrameFirst;

extern uint8_t   DetaTime;        //
extern uint8_t   BaseTime;        //

extern uint8_t   SaveTime;        //������������....
extern uint8_t   SendTime;        //SendTime��SaveTime֮����Ҫ�Ƚϣ�����һ��ע�ⲻҪ����޸�....
extern uint8_t   PackTime;        //PackTime��SaveTime֮����Ҫ�Ƚϣ�����һ��ע�ⲻҪ����޸�....


extern uint16_t   IMU_DMA_Num;

extern uint16_t   IMDMStime;

extern uint8_t    IMDHour;						//	 UTCʱ�䣺ʱ
extern uint8_t    IMDMinute;					//	 ��


extern uint8_t    IMDSecond;					//	 ��
extern uint8_t    IMDMSecond;        //   ����


extern uint8_t    IMU_Flag;
//-----------------------------------------
extern uint8_t    T1times;
extern uint16_t   T2times;
extern uint16_t   T3times;

extern uint8_t   Change_Data[50];   //
extern uint8_t   IMU_Num;           //��ȡIMU���ݵĴ���

void HandleGpsIniIMU(void);

void ProcessITOAH(uint8_t IMDATData);
void ProcessITOAM(uint16_t IMDATData);
void IMUData_Deal(uint8_t IGetData,uint8_t fisttime);
void IMUData_Deal_New(uint8_t IGetData,uint8_t fisttime);
void IMDAT_Deal(uint8_t fisttime);
void Change_SensorData(uint16_t SensorData);
void Change_Time(uint8_t SensorData);
uint8_t ProcessCheckIMU(uint8_t DNum);
void ProcessCheckToAIMU(uint8_t CheckNum);


void Save_IMUData(void);
void Save_DATData(void);
void Send_IMUData(void);

void  Get_BMI055_Data(void);
void  Pack_IMUData(void);
void  GetUTCTime(void);

void Data_Synch(void);

#endif

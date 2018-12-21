#ifndef __COMMDATA_H
#define __COMMDATA_H
#include "stdint.h"

#define SerialPutString(x) Serial_PutString((uint8_t*)(x)) ;


//----------------------------------------------------
extern uint8_t   Pass_Rx_Data[10];  //接收数组
extern uint8_t   Pass_Rx_Counter;
extern uint8_t   Pass_Rx_Max;

extern uint8_t   Up_Rx_Data[10];      //
extern uint8_t   Up_Rx_Counter;      // 
extern uint8_t   Up_Rx_Max;          // 

extern uint8_t   Key_Rx_Data[20];  //接收数组
extern uint8_t   Key_Rx_Counter;
extern uint8_t   Key_Rx_Max;

extern uint8_t  IMUTGetFlag;

extern uint8_t   Command_Tx_Max;	   //发送的最大字节个数；
extern uint8_t   Command_Rx_Max;	   //接受的最大字节个数；



extern uint8_t   Comm_Sort;		       //命令类型
extern uint8_t   Data_Com_Flag;      //1--数据，2--命令

extern uint8_t   AHRSComFlag;		     //
extern uint8_t   COM_Flag;		       //串口发送标志位


//----------------------------------------------------
extern uint8_t  TDM_TX_Data[400];
extern uint8_t  TDM_Rx_Data[60];	


extern uint8_t   High_DGet_Flag;       //频率设置参数
extern uint8_t   Buat_DGet_Flag;       //速率设置参数

extern uint8_t   GNGA_DGet_Flag;       //Gps获得标志
extern uint8_t   ZDA_DGet_Flag;        //Gps获得标志
extern uint8_t   SGPS_DGet_Flag;       //Gps获得标志


extern uint8_t   BD_DGet_Flag;         //BD设置标志
extern uint8_t   BD_GGet_Flag;         //BD获得标志

extern uint8_t   GST_TGet_Flag;        //打开关闭GST
extern uint8_t   ATT_DGet_Flag;        //打开关闭ATT
extern uint8_t   INS_DGet_Flag;        //打开关闭INS
extern uint8_t   GSV_DGet_Flag;        //打开关闭GSV

extern uint8_t    ANG_DGet_Flag;        //是否寻找
extern uint8_t    ANG_FGet_Flag;        //是否寻找
extern uint8_t    ANG_Lock_Flag;        //锁定
extern uint8_t    ANG_Kind_Flag;        //状态：0：固定版本，1：自适应版本
extern uint8_t    RSE_DGet_Flag;        //是否寻找	
extern uint8_t    RSE_FGet_Flag;        //重新搜索角度
extern uint8_t    RSE_SGet_Flag;        //重新搜索
extern uint8_t    POS_First_Flag;        //重新搜索角度

extern double     Lat_Back;             //经度
extern double     Lon_Back;             //维度


extern  int16_t   Pitch_Back;           //俯仰角
extern  int16_t   Roll_Back;            //横滚角
extern  int16_t   Head_Back;            //方向角

extern uint8_t    NorthSouthBack;       //
extern uint8_t    WestEastBack;         //

extern int16_t    MisAngleData;         //安装角度
extern int16_t    Ini0_RollBack;        //
extern int16_t    Ini0_PitchBack;       //

extern int16_t    Ini0_RollNew;         //
extern int16_t    Ini0_PitchNew;        //

extern uint8_t    Ini0_KindBack;
extern uint8_t    Ini0_KindNew;


extern uint8_t    Ins_Go_Flag;

extern uint8_t    Uart_Kind;
extern uint8_t    User_Kind;


extern uint8_t    Debug_Flag;
extern uint8_t    Up_On_Flag;         // 
extern uint8_t    Data_Sort;		       //数据类型


//低功耗测试用
extern uint8_t   Lowpow_Flag;
extern uint8_t   Lowpow_Sleep;

void SendALLData(void);
void SendData(void);
void Com2SendData(void);

void SendIMUData(void);
void IMU_DMA(uint16_t SecondTime);
void IMU_DMA_Send(void);
void Command_Handle(unsigned char Comamnd);
void HandleUpdata(unsigned char UCdata);
void HandleKeyOn(unsigned char KComamnd);


void HandlePassOn(unsigned char pComamnd);

uint16_t GetCrcData(void);
void SendVersion(void);
void Serial_PutString(uint8_t *s);

void HandleUserData(uint8_t UerData);

void ProcessVTOA(uint16_t VData);


#endif


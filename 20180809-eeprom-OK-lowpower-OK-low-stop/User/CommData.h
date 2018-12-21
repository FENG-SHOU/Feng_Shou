#ifndef __COMMDATA_H
#define __COMMDATA_H
#include "stdint.h"

#define SerialPutString(x) Serial_PutString((uint8_t*)(x)) ;


//----------------------------------------------------
extern uint8_t   Pass_Rx_Data[10];  //��������
extern uint8_t   Pass_Rx_Counter;
extern uint8_t   Pass_Rx_Max;

extern uint8_t   Up_Rx_Data[10];      //
extern uint8_t   Up_Rx_Counter;      // 
extern uint8_t   Up_Rx_Max;          // 

extern uint8_t   Key_Rx_Data[20];  //��������
extern uint8_t   Key_Rx_Counter;
extern uint8_t   Key_Rx_Max;

extern uint8_t  IMUTGetFlag;

extern uint8_t   Command_Tx_Max;	   //���͵�����ֽڸ�����
extern uint8_t   Command_Rx_Max;	   //���ܵ�����ֽڸ�����



extern uint8_t   Comm_Sort;		       //��������
extern uint8_t   Data_Com_Flag;      //1--���ݣ�2--����

extern uint8_t   AHRSComFlag;		     //
extern uint8_t   COM_Flag;		       //���ڷ��ͱ�־λ


//----------------------------------------------------
extern uint8_t  TDM_TX_Data[400];
extern uint8_t  TDM_Rx_Data[60];	


extern uint8_t   High_DGet_Flag;       //Ƶ�����ò���
extern uint8_t   Buat_DGet_Flag;       //�������ò���

extern uint8_t   GNGA_DGet_Flag;       //Gps��ñ�־
extern uint8_t   ZDA_DGet_Flag;        //Gps��ñ�־
extern uint8_t   SGPS_DGet_Flag;       //Gps��ñ�־


extern uint8_t   BD_DGet_Flag;         //BD���ñ�־
extern uint8_t   BD_GGet_Flag;         //BD��ñ�־

extern uint8_t   GST_TGet_Flag;        //�򿪹ر�GST
extern uint8_t   ATT_DGet_Flag;        //�򿪹ر�ATT
extern uint8_t   INS_DGet_Flag;        //�򿪹ر�INS
extern uint8_t   GSV_DGet_Flag;        //�򿪹ر�GSV

extern uint8_t    ANG_DGet_Flag;        //�Ƿ�Ѱ��
extern uint8_t    ANG_FGet_Flag;        //�Ƿ�Ѱ��
extern uint8_t    ANG_Lock_Flag;        //����
extern uint8_t    ANG_Kind_Flag;        //״̬��0���̶��汾��1������Ӧ�汾
extern uint8_t    RSE_DGet_Flag;        //�Ƿ�Ѱ��	
extern uint8_t    RSE_FGet_Flag;        //���������Ƕ�
extern uint8_t    RSE_SGet_Flag;        //��������
extern uint8_t    POS_First_Flag;        //���������Ƕ�

extern double     Lat_Back;             //����
extern double     Lon_Back;             //ά��


extern  int16_t   Pitch_Back;           //������
extern  int16_t   Roll_Back;            //�����
extern  int16_t   Head_Back;            //�����

extern uint8_t    NorthSouthBack;       //
extern uint8_t    WestEastBack;         //

extern int16_t    MisAngleData;         //��װ�Ƕ�
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
extern uint8_t    Data_Sort;		       //��������


//�͹��Ĳ�����
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


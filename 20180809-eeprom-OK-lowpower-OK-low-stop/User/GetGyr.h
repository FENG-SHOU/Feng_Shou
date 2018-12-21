#ifndef __GETGYR_H
#define __GETGYR_H

#include "stm32f4xx.h" 

extern int16_t  GyrBias_X;       // �����ǵĸ�����ƫ��ʵʱ����   
extern int16_t  GyrBias_Y;       // �����ǵĸ�����ƫ��ʵʱ����   
extern int16_t  GyrBias_Z;       // �����ǵĸ�����ƫ��ʵʱ����

extern int16_t  GyrScale_X;      // �����ǵ�X��������  store
extern int16_t  GyrScale_Y;      // �����ǵ�Y��������  store
extern int16_t  GyrScale_Z;      // �����ǵ�Z��������  store 

extern int16_t  XGyrBias;        // �����ǵĸ�����ƫ��   store
extern int16_t  YGyrBias;        // �����ǵĸ�����ƫ��   store
extern int16_t  ZGyrBias;        // �����ǵĸ�����ƫ��   store

extern int16_t  XGyrScale;       // �����ǵ�X��������  store
extern int16_t  YGyrScale;       // �����ǵ�Y��������  store
extern int16_t  ZGyrScale;       // �����ǵ�Z��������  store 

extern float  floPitchGyr;		  //???��X
extern float  floRollGyr;		    //???��Y
extern float  floYawGyr;		    //???��Z

extern float   floYawAngle;


//---------------------------------------------------
//
//---------------------------------------------------

//extern uint8_t BMI055G_Rx[6];	 //
extern uint8_t BMI160G_Rx[6];

extern int16_t  Gyr_X;			 //������X������
extern int16_t  Gyr_Y;			 //������Y������
extern int16_t  Gyr_Z;			 //������Z������


extern int16_t intPitchGyr;	   // 
extern int16_t intRollGyr;		 // 
extern int16_t intYawGyr;		   // 

extern uint8_t  IMU_Kind;


extern uint8_t  GpsKind;
extern uint8_t ProductKind;


extern uint8_t Ddx_data[8];		 
extern uint8_t Ttx_data[8];	
extern uint8_t error_code;



void   Gyr_Ini(void);
//void   BMI055G_Ini(void);
void   Get_Yaw_Data(void);

void   Get_Rate_Data(void);
//void   Get_BMI055G_Data(void);
void   Get_BMI160G_Data(void);
void   Get_Yaw_Data(void);
float  ConvertorGyrData(int GyrData,int Bias,int GyrScale,char Reverse);

int16_t Checkout_PGyr(int16_t g_data);
int16_t Checkout_RGyr(int16_t g_data);
int16_t Checkout_YGyr(int16_t g_data);

#endif 

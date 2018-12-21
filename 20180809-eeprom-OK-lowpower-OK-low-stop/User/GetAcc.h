#ifndef __GETACC_H
#define __GETACC_H

#include "stm32f4xx.h" 


extern uint8_t User_Kind;
extern uint8_t DIni_Flag;
//-----------------------------------------
extern int16_t XAccBias;
extern int16_t XAccScale;
extern int16_t XAccFScale;

//-----------------------------------
extern int16_t YAccBias;
extern int16_t YAccScale;
extern int16_t YAccFScale;
//-----------------------------------
extern int16_t ZAccBias;
extern int16_t ZAccScale;
extern int16_t ZAccFScale;

extern float   floPitchAcc;
extern float   floRollAcc;
extern float   floAliAcc;


extern float   floPitchAngle;
extern float   floRollAngle;


//------------------------------------------
//
//------------------------------------------
//extern uint8_t BMI055A_Rx[6];
extern uint8_t  BMI160A_Rx[6];


extern int16_t ACC_X;
extern int16_t ACC_Y;
extern int16_t ACC_Z;

extern int16_t  intPitchAcc;		   //
extern int16_t  intRollAcc;			 //
extern int16_t  intAliAcc;			   //


extern int16_t  intPitchAngle;	   //基于加速度计的俯仰角
extern int16_t  intRollAngle;	     //基于加速度计的横滚角
extern int16_t  intYawAngle;       //基于角速度的航向角

extern uint8_t  IMU_Kind;
extern uint8_t ProductKind;

extern	float pi;
extern  float Scale;

extern uint8_t Ddx_data[8];		 
extern uint8_t Ttx_data[8];	
extern uint8_t error_code;

extern uint8_t ANG_Kind_Flag;

//void BMI055A_Ini(void);
void BMI160_Ini(void);
void Interrupt_Any_motion(void);
void Interrupt_Tap_sensing(void);
void Interrupt_Chose(void);
//void Get_BMI055A_Data(void);
void  Get_BMI160A_Data(void);
void Get_Acc_XYZ(void);
void Get_Angle_XYZ(void);
void Acc_Ini(void);

int  Convert_Acc_Data(uint8_t High,uint8_t Low);

float ConvertorPAcc(int AccData,char Reverse);
float ConvertorRAcc(int AccData,char Reverse);
float ConvertorZAcc(int AccData,char Reverse);

int16_t Checkout_p2g(int16_t g_data);
int16_t Checkout_a2g(int16_t g_data);
int16_t Checkout_r2g(int16_t g_data);

float ConvertorPAngle(int16_t XAcc,uint8_t Reverse);
float ConvertorRAngle(int16_t YAccData,int16_t ZAccData,uint8_t Reverse);
float Checkout_Pi(float C_data);
float Checkout_2Pi(float C_data);
#endif 

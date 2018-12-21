#ifndef __GETGYR_H
#define __GETGYR_H

#include "stm32f4xx.h" 

extern int16_t  GyrBias_X;       // 陀螺仪的俯仰轴偏置实时变量   
extern int16_t  GyrBias_Y;       // 陀螺仪的俯仰轴偏置实时变量   
extern int16_t  GyrBias_Z;       // 陀螺仪的俯仰轴偏置实时变量

extern int16_t  GyrScale_X;      // 陀螺仪的X轴标称因子  store
extern int16_t  GyrScale_Y;      // 陀螺仪的Y轴标称因子  store
extern int16_t  GyrScale_Z;      // 陀螺仪的Z轴标称因子  store 

extern int16_t  XGyrBias;        // 陀螺仪的俯仰轴偏置   store
extern int16_t  YGyrBias;        // 陀螺仪的俯仰轴偏置   store
extern int16_t  ZGyrBias;        // 陀螺仪的俯仰轴偏置   store

extern int16_t  XGyrScale;       // 陀螺仪的X轴标称因子  store
extern int16_t  YGyrScale;       // 陀螺仪的Y轴标称因子  store
extern int16_t  ZGyrScale;       // 陀螺仪的Z轴标称因子  store 

extern float  floPitchGyr;		  //???棗X
extern float  floRollGyr;		    //???棗Y
extern float  floYawGyr;		    //???棗Z

extern float   floYawAngle;


//---------------------------------------------------
//
//---------------------------------------------------

//extern uint8_t BMI055G_Rx[6];	 //
extern uint8_t BMI160G_Rx[6];

extern int16_t  Gyr_X;			 //陀螺仪X轴数据
extern int16_t  Gyr_Y;			 //陀螺仪Y轴数据
extern int16_t  Gyr_Z;			 //陀螺仪Z轴数据


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

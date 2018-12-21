#include "GetAcc.h"
#include "delay.h"
#include "spi.h"
#include "math.h"

void CoordChangeA(void)
{
   int16_t	 ACC_XT,ACC_YT,ACC_ZT;			       //加速度计X轴数据
	
	 if(ProductKind==1)
	 {
		ACC_XT =  Convert_Acc_Data(BMI160A_Rx[1],BMI160A_Rx[0]);
		ACC_YT = -Convert_Acc_Data(BMI160A_Rx[3],BMI160A_Rx[2]);
		ACC_ZT = -Convert_Acc_Data(BMI160A_Rx[5],BMI160A_Rx[4]);	 

	 //------------------------------------ 	
   if(IMU_Kind==1)
	 {
		ACC_X =   ACC_XT;
		ACC_Y =   ACC_YT;	
		ACC_Z =   ACC_ZT;
	 }
	 else if(IMU_Kind==2)
	 {
		ACC_X =  -ACC_YT;
		ACC_Y =   ACC_XT;	
		ACC_Z =   ACC_ZT;
	 }
	 else if(IMU_Kind==3)
	 {
		ACC_X =  -ACC_XT;
		ACC_Y =  -ACC_YT;	
		ACC_Z =   ACC_ZT;
	 }
	 else if(IMU_Kind==4)
	 {
		ACC_X =   ACC_YT;
		ACC_Y =  -ACC_XT;	
		ACC_Z =   ACC_ZT;
	 }
	//------------------------------------ 
	 else if(IMU_Kind==5)
	 {
		ACC_X =   ACC_XT;
		ACC_Y =  -ACC_YT;	
		ACC_Z =  -ACC_ZT;
	 }
	 else if(IMU_Kind==6)
	 {
		ACC_X =   ACC_YT;
		ACC_Y =   ACC_XT;	
		ACC_Z =  -ACC_ZT;
	 }
	 else if(IMU_Kind==7)
	 {
		ACC_X =  -ACC_XT;
		ACC_Y =   ACC_YT;	
		ACC_Z =  -ACC_ZT;
	 }
	 else if(IMU_Kind==8)
	 {
		ACC_X =  -ACC_YT;
		ACC_Y =  -ACC_XT;	
		ACC_Z =  -ACC_ZT;
	 }
	//------------------------------------ 
	 else
	 {
		ACC_X =  ACC_XT;
		ACC_Y =  ACC_YT;	
		ACC_Z =  ACC_ZT; 
	 }
  }
	else
	{
		ACC_XT = -Convert_Acc_Data(BMI160A_Rx[1],BMI160A_Rx[0]);
		ACC_YT = -Convert_Acc_Data(BMI160A_Rx[3],BMI160A_Rx[2]);
		ACC_ZT =  Convert_Acc_Data(BMI160A_Rx[5],BMI160A_Rx[4]); 

   if(IMU_Kind==9)     //模块  X轴向后....
	 {
		ACC_X = -ACC_XT;
		ACC_Y = -ACC_YT;
		ACC_Z =  ACC_ZT;
   }
	  else if(IMU_Kind==8)    //模块  X轴向右....
	 {
	 	ACC_Y =  ACC_XT;
		ACC_X = -ACC_YT;
		ACC_Z =  ACC_ZT;	 
	 } 
   else if(IMU_Kind==6)    //模块  X轴向左..
   {
	 	ACC_Y = -ACC_XT;
		ACC_X =  ACC_YT;
		ACC_Z =  ACC_ZT;	 
	 }
   else// if(IMU_Kind==7)    //模块  X轴向前.....
	 {
	 	ACC_X =  ACC_XT;
		ACC_Y =  ACC_YT;
		ACC_Z =  ACC_ZT;	 
	 }
	}

	if(ANG_Kind_Flag==0)     //如果自适应，则
	{	
		ACC_XT =  Convert_Acc_Data(BMI160A_Rx[1],BMI160A_Rx[0]);
		ACC_YT = -Convert_Acc_Data(BMI160A_Rx[3],BMI160A_Rx[2]);
		ACC_ZT = -Convert_Acc_Data(BMI160A_Rx[5],BMI160A_Rx[4]);	 
	
		ACC_X = -ACC_XT;
		ACC_Y = -ACC_YT;
		ACC_Z =  ACC_ZT;
	}
} 
	 
	 
	 
void Acc_Ini(void)
{
	if(User_Kind==2)
	{
 	XAccBias=0;
	XAccScale    = 512;
  XAccFScale   = 512;
	
	YAccBias=0;
	YAccScale    = 512;	
  YAccFScale   = 512;	
	
	ZAccBias=0;
	ZAccScale    = 512;
  ZAccFScale   = 512;		
	}
	else
	{
 	XAccBias=0;
	XAccScale    = 1024;
  XAccFScale   = 1024;
	
	YAccBias=0;
	YAccScale    = 1024;	
  YAccFScale   = 1024;	
	
	ZAccBias=0;
	ZAccScale    = 1024;
  ZAccFScale   = 1024;
	}

}

uint8_t ID;

void BMI160_Ini(void)
{
	SETBMI160;
  delay_us(5);
	RESETBMI160;
	delay_us(50);
  SPI_ADS_SendData(0x80);	  //注意：地址为0x00,读取0x80
	ID=SPI_ADS_SendData(0x00);//读ID测试
	
	SETBMI160;	
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x7e);
	SPI_ADS_SendData(0x11);//Acc 正常模式
	
	SETBMI160;
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x7E);
	SPI_ADS_SendData(0x15);//Gro 正常模式
	
	
	SETBMI160;
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x40);//ACC_CONF
	SPI_ADS_SendData(0x2c);//1.6hz？
	
	SETBMI160;
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x41);//加速度计范围
	SPI_ADS_SendData(0x03);//2g  3-2g,5-4g,8-8g,12-16g
	
	SETBMI160;
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x42);//GYR_CONF
	SPI_ADS_SendData(0x2C);//1.6hz?
	
	SETBMI160;
	delay_us(500);
	RESETBMI160;
	delay_us(10);
	SPI_ADS_SendData(0x42);//陀螺仪范围
	SPI_ADS_SendData(0x03);//250dps  0-2000g;1-1000g,2-500g,3-250g;4-125g 
	
}

//检测运动进入中断配置
void Interrupt_Any_motion(void) //INT 1 PA0
{
	  SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x50);//INT_EN0
		SPI_ADS_SendData(0x07);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x53);//INT_OUT_CTRL
		SPI_ADS_SendData(0x0A);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x54);//INT_OUT_LATCH
		SPI_ADS_SendData(0x10);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x55);//INT_MAP2
		SPI_ADS_SendData(0x04);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x5f);//INT_MOTION
		SPI_ADS_SendData(0x02);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x60);//INT_MOTION
		SPI_ADS_SendData(0x28);//
}
//轻敲配置，INT1，连到PA0
void Interrupt_Tap_sensing(void)
{
	  SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x50);//INT_EN0
		SPI_ADS_SendData(0x30);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x53);//INT_OUT_CTRL
		SPI_ADS_SendData(0x0a);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x54);//INT_OUT_LATCH
		SPI_ADS_SendData(0x12);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x55);//INT_MAP2
		SPI_ADS_SendData(0x20);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x63);//INT_TAP0
		SPI_ADS_SendData(0x01);//
		
		SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x64);//INT_TAP1
		SPI_ADS_SendData(0x0A);//
}

void Interrupt_Close(void)
{
	  SETBMI160;
		delay_us(500);
		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x50);//INT_EN0
		SPI_ADS_SendData(0x00);//
}

void  Get_BMI160A_Data(void)
{	
	SETBMI160;
	delay_us(10);
	RESETBMI160;
	
	SPI_ADS_SendData(0x92);   	//注：地址0x12读取0x92
	BMI160A_Rx[0]=  SPI_ADS_SendData(0x00);
  BMI160A_Rx[1] = SPI_ADS_SendData(0x00);
  BMI160A_Rx[2] = SPI_ADS_SendData(0x00);
  BMI160A_Rx[3] = SPI_ADS_SendData(0x00);
  BMI160A_Rx[4] = SPI_ADS_SendData(0x00);
  BMI160A_Rx[5] = SPI_ADS_SendData(0x00);
	SETBMI160;
	delay_us(500);

  CoordChangeA();
}

//void BMI055A_Ini(void)		 //量程设为+/-2g,采样速度为2KHz
//{
//	uint8_t AccGet,AccSet;
//	
//		DISABLE_BMI055G_SPI;	 
//		DISABLE_BMI055A_SPI;

//	  delay_us(5);	
//	
//		ENABLE_BMI055A_SPI;	      //CSB1_Acc-->PB5输出低电平——片选BMX055_Acc;
//		delay_us(30); 
//		SPI_ADS_SendData(0x14); //写地址为0x14的寄存器（BGW_SOFTRESET）
//		SPI_ADS_SendData(0xB6); //对传感器进行软件复位以恢复默认设置——标准模式（P58）;量程为+/-2g;SPI接口采用4线制(P74);
//		delay_us(50);	
//		DISABLE_BMI055A_SPI;      //data_high_bw=0——加速度数据为滤波后数据；SPI为8位模式时shadow_dis=1——不能保证数据完整性-->这两位均需重新设置

//  	delay_ms(3);		 

//	
//	
//	  //------------------------------------
//		if(User_Kind==0x02)
//		{		 
//		  AccSet=0x05;           //3-2g,5-4g,8-8g,12-16g
//		}
//		else
//		{
//      AccSet=0x03;           //3-2g,5-4g,8-8g,12-16g
//		}			
//	 
//	  do
//		{
//			ENABLE_BMI055A_SPI;			            //片选BMX055_Gyr
//		  delay_us(30); 
//			if(User_Kind==0x02)
//			{
//			 SPI_ADS_SendData(0x0F);           //设置加速度计的量程
//			 SPI_ADS_SendData(0x05);           //3-2g,5-4g,8-8g,12-16g
//			}
//			else
//			{
//			 SPI_ADS_SendData(0x0F);           //设置加速度计的量程
//			 SPI_ADS_SendData(0x03);           //3-2g,5-4g,8-8g,12-16g
//			}
//			delay_us(50);	
//			DISABLE_BMI055A_SPI;
//			
//			delay_ms(1);			
//			
//			ENABLE_BMI055A_SPI;	                  //片选BMX055_Acc
//			delay_us(30); 
//			SPI_ADS_SendData(0x8F);               //读地址为0x02的寄存器（ACCD_X_LSB）
//			AccGet =SPI_ADS_SendData(0x00);       //读寄存器ACCD_X_LSB			
//			delay_us(30); 
//			DISABLE_BMI055A_SPI;
//		
//	  }while(AccGet!=AccSet);
//    //------------------------------------
//		
//  	delay_us(5);		 
//		ENABLE_BMI055A_SPI;
//		delay_us(30); 
//		SPI_ADS_SendData(0x13);           //写地址为0x13的寄存器（ACCD_HBW）
//		SPI_ADS_SendData(0x00);           //加速度数据为滤波的数据,保存数据完整性
//		delay_us(50);	
//		DISABLE_BMI055A_SPI;

//  	delay_us(5);
//		ENABLE_BMI055A_SPI;
//		delay_us(30); 
//		SPI_ADS_SendData(0x10);					//写地址为0x10的寄存器（PMU_BW）
//		SPI_ADS_SendData(0x08);					//滤波带宽为7.81HZ，最小  P55
//		delay_us(50);	
//		DISABLE_BMI055A_SPI;
//}



//void  Get_BMI055A_Data(void)
//{
//		DISABLE_BMI055G_SPI;    	 
//		DISABLE_BMI055A_SPI;
//	  delay_us(100);		
//		
//		ENABLE_BMI055A_SPI;	                  //片选BMX055_Acc 	  
//		SPI_ADS_SendData(0x82);               //读地址为0x02的寄存器（ACCD_X_LSB）

//		BMI055A_Rx[0] =SPI_ADS_SendData(0x00);//读寄存器ACCD_X_LSB
//		BMI055A_Rx[1] =SPI_ADS_SendData(0x00);//读寄存器ACCD_X_MSB

//		BMI055A_Rx[2] =SPI_ADS_SendData(0x00);//读寄存器ACCD_Y_LSB
//		BMI055A_Rx[3] =SPI_ADS_SendData(0x00);//读寄存器ACCD_Y_MSB

//		BMI055A_Rx[4] =SPI_ADS_SendData(0x00);//读寄存器ACCD_Z_LSB
//		BMI055A_Rx[5] =SPI_ADS_SendData(0x00);//读寄存器ACCD_Z_MSB

//		DISABLE_BMI055A_SPI;

//    CoordChangeA();
//	
//}




int Convert_Acc_Data(uint8_t High,uint8_t Low)
{
   int16_t Temp=0;

   if(High>0x7F)      //High最高位为1——加速度值为负值  
    {
      Temp = High*256+Low;
      Temp = Temp-1;
      Temp =~Temp;
      Temp = Temp>>4;
      Temp = 0-Temp; //机器中负数以补码表示——将补码表示为原码
    }
   else
    {
      Temp=High*256+Low;
      Temp=Temp>>4;
    }
    return Temp; 	
} 





//---------------------------------
//三轴加速度计转换为物理量
//---------------------------------
void Get_Acc_XYZ(void)
{

   floPitchAcc = ConvertorPAcc(ACC_X,0);
   floRollAcc  = ConvertorRAcc(ACC_Y,0);
   floAliAcc   = ConvertorZAcc(ACC_Z,0);

	
   if(Ttx_data[0]==Ddx_data[0])
	 {
   intPitchAcc = Checkout_p2g(floPitchAcc*1000);	   //加速度(整形)
   intRollAcc  = Checkout_r2g(floRollAcc*1000);     //加速度
   intAliAcc   = Checkout_a2g(floAliAcc*1000);     //加速度
	 }
	 else
	 {
	  intPitchAcc = Checkout_p2g(floPitchAcc*2000);	   //加速度(整形)
    intRollAcc  = Checkout_r2g(floRollAcc*2000);     //加速度
    intAliAcc   = Checkout_a2g(floAliAcc*2000);     //加速度	 
	 }



}

//--------------------------------------------------
// 俯仰轴加速度计转换
//--------------------------------------------------
 float ConvertorPAcc(int AccData,char Reverse) 
 {
  float AcceratorData; 
  float TAcce; 
  float Tbias,Zbias,Fbias; 
  float Zscale,Fscale,ZZscale,ZFscale;  

  TAcce = AccData;
  Tbias = XAccBias; 

  Zscale  = XAccScale;
  Fscale  = XAccFScale;

 
  if(TAcce>Tbias)
	{
    AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	  AcceratorData = (TAcce-Tbias)/Fscale;	 
	}
	
   return 	 AcceratorData;
}
 
//--------------------------------------------------
// 横滚加速度计转换
//--------------------------------------------------
 float ConvertorRAcc(int AccData,char Reverse) 
 {
  float AcceratorData; 
  float TAcce; 
  float Tbias,Zbias,Fbias; 
  float Zscale,Fscale,ZZscale,ZFscale;  

  TAcce = AccData;
  Tbias = YAccBias;   
	 
  Zscale  = YAccScale;
  Fscale  = YAccFScale;
 
  if(TAcce>Tbias)
	{
    AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	  AcceratorData = (TAcce-Tbias)/Fscale;	 
	}

  if(Reverse)
	{
	  AcceratorData = -AcceratorData;
	}
	
   return 	 AcceratorData;
}

//--------------------------------------------------
// 垂直轴加速度计转换
//--------------------------------------------------
 float ConvertorZAcc(int AccData,char Reverse) 
 {
  float AcceratorData;

	float TAcce;
	float Tbias;
	float Zscale,Fscale; 
  
 	TAcce=AccData;
	Tbias = ZAccBias;
	 
	Zscale  = ZAccScale;	
	Fscale  = ZAccFScale;

   
  if(TAcce>Tbias)
	{
   AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	 AcceratorData = (TAcce-Tbias)/Fscale;	 
	}

  if(Reverse)
	{
	 AcceratorData =-AcceratorData;
	}
  
   return AcceratorData;    
}
//**********************************************
//-----------------------------------------
//int Checkout_Pi(float C_data)
//------------------------------------------
int16_t Checkout_r2g(int16_t g_data)
{
 static int8_t First=1;
 static int16_t g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>2000)
     g_data=g_data_Back;
   else if(g_data<-2000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

   return g_data ;
}

//-----------------------------------------
//int Checkout_Pi(float C_data)
//------------------------------------------
int16_t Checkout_p2g(int16_t g_data)
{
 static int8_t First=1;
 static int16_t g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>2000)          //????2gJ????2g
     g_data=g_data_Back;
   else if(g_data<-2000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

   return g_data ;
}

//-----------------------------------------
//int Checkout_Pi(float C_data)
//------------------------------------------
int16_t Checkout_a2g(int16_t g_data)
{
 static int8_t First=1;
 static int16_t g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>2000)
     g_data=g_data_Back;
   else if(g_data<-2000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

   return g_data ;
}


void Get_Angle_XYZ(void)
{

	
	floPitchAngle = ConvertorPAngle(intPitchAcc,0);					       //???
	floRollAngle  = ConvertorRAngle(intRollAcc,-intAliAcc,0);  		 //???
	
  floRollAngle   = Checkout_Pi(floRollAngle); 
  floPitchAngle  = Checkout_2Pi(floPitchAngle); 

  intPitchAngle = floPitchAngle*Scale*10;
  intRollAngle  = floRollAngle*Scale*10;  
}

//*****************************************
//-----------------------------------------
//int Checkout_Pi(float C_data)
//------------------------------------------
float Checkout_Pi(float C_data)
{
   if(C_data>pi)
     C_data=pi;
   else if(C_data<-pi)
     C_data=-pi; 

   return C_data;
}

//-----------------------------------------
//int Checkout_2Pi(float C_data)
//------------------------------------------
float Checkout_2Pi(float C_data)
{
   if(C_data>pi/2)
     C_data=pi/2;
   else if(C_data<-pi/2)
     C_data=-pi/2; 

   return C_data;
}
//------------------------------------------------
//??????????
//------------------------------------------------
float ConvertorPAngle(int16_t XAcc,uint8_t Reverse)
{
 float AngleTemp;

  if(XAcc>1000)
  {
   XAcc=1000;  
  }
  else if(XAcc<-1000)
  {
   XAcc=-1000;  
  }

 AngleTemp=asin(XAcc/1000.0f);

 if(Reverse)
 {
 AngleTemp=-AngleTemp; 
 }
 return AngleTemp;
}

//------------------------------------------------------------------------
//??????????
//------------------------------------------------------------------------
float ConvertorRAngle(int16_t YAccData,int16_t ZAccData,uint8_t Reverse)
{ 
    static  float AcceratorData;

	if(YAccData>=1000)
	{
  	  YAccData=1000;
	}
	else if(YAccData<=-1000)
	{
  	  YAccData=-1000;
	}
   
	if(ZAccData>=1000)
	{
  	  ZAccData=1000;
	}
	else if(ZAccData<=-1000)
	{
  	  ZAccData=-1000;
	}
   //---------------------------------------

  AcceratorData=atan2(YAccData/1000.0f,ZAccData/1000.0f);
   
	 
	if(Reverse)
	{
     AcceratorData=-AcceratorData;
	}
   
   return AcceratorData;   
}

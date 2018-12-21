#include "GetGyr.h"
#include "GetAcc.h"
#include "spi.h"
#include "Delay.h"
#include "commdata.h"



void CoordChangeG(void)
{
	int16_t  Gyr_XT,Gyr_YT,Gyr_ZT;			      //陀螺仪X轴数据
	
	 if(ProductKind==1)
	 {
	  Gyr_XT=  (BMI160G_Rx[1]*256+BMI160G_Rx[0]);
		Gyr_YT= -(BMI160G_Rx[3]*256+BMI160G_Rx[2]);
		Gyr_ZT= -(BMI160G_Rx[5]*256+BMI160G_Rx[4]);	
	
	 
		//------------------------------------ 	
   if(IMU_Kind==1)
	 {
		Gyr_X =   Gyr_XT;
		Gyr_Y =   Gyr_YT;	
		Gyr_Z =   Gyr_ZT;
	 }
	 else if(IMU_Kind==2)
	 {
		Gyr_X =  -Gyr_YT;
		Gyr_Y =   Gyr_XT;	
		Gyr_Z =   Gyr_ZT;
	 }
	 else if(IMU_Kind==3)
	 {
		Gyr_X =  -Gyr_XT;
		Gyr_Y =  -Gyr_YT;	
		Gyr_Z =   Gyr_ZT;
	 }
	 else if(IMU_Kind==4)
	 {
		Gyr_X =   Gyr_YT;
		Gyr_Y =  -Gyr_XT;	
		Gyr_Z =   Gyr_ZT;
	 }
	//------------------------------------ 
	 else if(IMU_Kind==5)
	 {
		Gyr_X =   Gyr_XT;
		Gyr_Y =  -Gyr_YT;	
		Gyr_Z =  -Gyr_ZT;
	 }
	 else if(IMU_Kind==6)
	 {
		Gyr_X =   Gyr_YT;
		Gyr_Y =   Gyr_XT;	
		Gyr_Z =  -Gyr_ZT;
	 }
	 else if(IMU_Kind==7)
	 {
		Gyr_X =  -Gyr_XT;
		Gyr_Y =   Gyr_YT;	
		Gyr_Z =  -Gyr_ZT;
	 }
	 else if(IMU_Kind==8)
	 {
		Gyr_X =  -Gyr_YT;
		Gyr_Y =  -Gyr_XT;	
		Gyr_Z =  -Gyr_ZT;
	 }
	//------------------------------------ 
	 else
	 {
		Gyr_X =  Gyr_XT;
		Gyr_Y =  Gyr_YT;	
		Gyr_Z =  Gyr_ZT; 
	 }
  }
	else
	{
		Gyr_XT = -(BMI160G_Rx[1]*256+BMI160G_Rx[0]);
		Gyr_YT = -(BMI160G_Rx[3]*256+BMI160G_Rx[2]);
		Gyr_ZT =  (BMI160G_Rx[5]*256+BMI160G_Rx[4]);	 
	
	 if(IMU_Kind==9)    //模块
	 {
		Gyr_X=  -Gyr_XT;
		Gyr_Y=  -Gyr_YT;
		Gyr_Z=   Gyr_ZT;
   }
	  else if(IMU_Kind==8)    //              
	 {
	  Gyr_Y=   Gyr_XT;
		Gyr_X=  -Gyr_YT;
		Gyr_Z=   Gyr_ZT;
	 }
	 else if(IMU_Kind==6)    //X轴超后
	 {
	  Gyr_Y= -Gyr_XT;
		Gyr_X=  Gyr_YT;
		Gyr_Z=  Gyr_ZT;
	 }
	 else //if(IMU_Kind==7)    //X轴超前--标准安装
	 {
	  Gyr_X=  Gyr_XT;
		Gyr_Y=  Gyr_YT;
		Gyr_Z=  Gyr_ZT;	 
	 }
	}	 
	
	if(ANG_Kind_Flag==0)       //如果自适应，则
	{
	   Gyr_XT=  (BMI160G_Rx[1]*256+BMI160G_Rx[0]);
		Gyr_YT= -(BMI160G_Rx[3]*256+BMI160G_Rx[2]);
		Gyr_ZT= -(BMI160G_Rx[5]*256+BMI160G_Rx[4]);	 
	
		Gyr_X=  -Gyr_XT;
		Gyr_Y=  -Gyr_YT;
		Gyr_Z=   Gyr_ZT;
	}
}	 
	 
	 
	 
void Gyr_Ini(void)
{
   GyrBias_X  = 0;
   GyrBias_Y  = 0;
   GyrBias_Z  = 0;

   GyrScale_X = 61;
   GyrScale_Y = 61;	
   GyrScale_Z = 61;	
}

void  Get_BMI160G_Data(void)
{

		RESETBMI160;
		delay_us(10);
		SPI_ADS_SendData(0x8c);//注：地址0x0c读取0x8c
		BMI160G_Rx[0] = SPI_ADS_SendData(0x00);
		BMI160G_Rx[1] = SPI_ADS_SendData(0x00);
		BMI160G_Rx[2] = SPI_ADS_SendData(0x00);
		BMI160G_Rx[3] = SPI_ADS_SendData(0x00);
		BMI160G_Rx[4] = SPI_ADS_SendData(0x00);
		BMI160G_Rx[5] = SPI_ADS_SendData(0x00);
		SETBMI160;
		delay_us(500);
	
    CoordChangeG();
}



//void BMI055G_Ini(void)		 //量程设为+/-250dps,ODR=2KHz
//{
//		uint8_t GyrGet,GyrSet=0x03;
//	
//		DISABLE_BMI055G_SPI;	 
//		DISABLE_BMI055A_SPI;
//	  delay_us(30);	
//	
//		ENABLE_BMI055G_SPI;	     //CSB1_Acc-->PB5输出低电平——片选BMX055_Acc;
//		delay_us(5); 
//		SPI_ADS_SendData(0x14); //写地址为0x14的寄存器（BGW_SOFTRESET）
//		SPI_ADS_SendData(0xB6); //对传感器进行软件复位以恢复默认设置——标准模式（P100）;SPI接口采用4线制(P113)；量程2000dps(P99)
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;     //data_high_bw=0——加速度数据为滤波后数据；SPI为8位模式时shadow_dis=1——不能保证数据完整性-->这两位均需重新设置
//		 		
//		do
//		{
//			delay_ms(3);
//			ENABLE_BMI055G_SPI;			             //片选BMX055_Gyr
//			delay_us(30); 
//			SPI_ADS_SendData(0x0F);              //写地址为0x0f的寄存器（RANGE）
//			SPI_ADS_SendData(0x03);              //0-2000g;1-1000g,2-500g,3-250g;4-125g  
//			delay_us(50);	
//			DISABLE_BMI055G_SPI;	
//			
//			delay_ms(1);
//			
//			ENABLE_BMI055G_SPI;	                  //片选BMX055_Acc
//			SPI_ADS_SendData(0x8F);               //读地址为0x02的寄存器（ACCD_X_LSB）
//			GyrGet =SPI_ADS_SendData(0x00);       //读寄存器ACCD_X_LSB			
//			DISABLE_BMI055G_SPI;
//		
//	  }while(GyrGet!=GyrSet);	
//	

//  	delay_us(5);		
//		ENABLE_BMI055G_SPI;
//		delay_us(30); 
//   	SPI_ADS_SendData(0x13);//写地址为0x13的寄存器（RATE_HBW）
//		SPI_ADS_SendData(0x00);//加速度数据为滤波后的数据；保证数据完整性
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;

//  	delay_us(5);
//		ENABLE_BMI055G_SPI;
//		delay_us(30); 
//		SPI_ADS_SendData(0x10);//写地址为0x10的寄存器（BW）
//		SPI_ADS_SendData(0x05);//滤波带宽为12HZ，最小  P100
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;

//}

//void  Get_BMI055G_Data(void)
//{
//		DISABLE_BMI055A_SPI;	 
//		DISABLE_BMI055G_SPI;
//    delay_us(100);	
//		 
//		ENABLE_BMI055G_SPI;                   //片选BMX055_Gyr	
//		SPI_ADS_SendData(0x82);               //读地址为0x02的寄存器（RATE_X_LSB）

//		BMI055G_Rx[0] =SPI_ADS_SendData(0x00);//读寄存器RATE_X_LSB
//		BMI055G_Rx[1] =SPI_ADS_SendData(0x00);//读寄存器RATE_X_MSB

//		BMI055G_Rx[2] =SPI_ADS_SendData(0x00);//读寄存器RATE_Y_LSB
//		BMI055G_Rx[3] =SPI_ADS_SendData(0x00);//读寄存器RATE_Y_MSB

//		BMI055G_Rx[4] =SPI_ADS_SendData(0x00);//读寄存器RATE_Z_LSB
//		BMI055G_Rx[5] =SPI_ADS_SendData(0x00);//读寄存器RATE_Z_MSB

//		DISABLE_BMI055G_SPI;

//    CoordChangeG();
//	 

//}

void Get_Rate_Data(void)
{


 floPitchGyr = ConvertorGyrData(Gyr_Y,GyrBias_Y,GyrScale_Y,0);   //去除偏置后的角速度
 floRollGyr  = ConvertorGyrData(Gyr_X,GyrBias_X,GyrScale_X,0);
 floYawGyr   = ConvertorGyrData(Gyr_Z,GyrBias_Z,GyrScale_Z,0);


 intPitchGyr = Checkout_PGyr(floPitchGyr*10);
 intRollGyr  = Checkout_RGyr(floRollGyr*10);
 intYawGyr   = Checkout_YGyr(floYawGyr*10);

}

//--------------------------------
//void Get_Yaw_Data(void)
//--------------------------------
void Get_Yaw_Data(void)
{
 if(Up_On_Flag)
 {	
  floYawAngle = floYawAngle + floYawGyr*0.02f;
 
  if(floYawAngle>180)
   floYawAngle = floYawAngle-360;	 
  else if(floYawAngle<-180)
	 floYawAngle = floYawAngle+360;	
 
  intYawAngle = floYawAngle*10;
 }
 
}

//----------------------------------------------------------------------------------
//角速度信号转换
//无论什么量程，均转为xx度/秒
//----------------------------------------------------------------------------------
float  ConvertorGyrData(int GyrData,int Bias,int GyrScale,char Reverse)
{
		float RateAngle;
		volatile float GyrDataTemp;
		float Tbias;
		float GyrScaleData;

		GyrDataTemp = GyrData/1.0f; 
		Tbias       = Bias;


		GyrScaleData= GyrScale/8000.0f;

		RateAngle   = (GyrData-Tbias)*GyrScaleData;


		if(Reverse)
		{
			RateAngle=-RateAngle;
		}

		return RateAngle;

}


//*****************************************
//-----------------------------------------
//int16_t Checkout_PGyr(int16_t g_data)
//------------------------------------------
int16_t Checkout_PGyr(int16_t g_data)
{
 static int First=1;
 static int g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>20000)
     g_data=g_data_Back;
   else if(g_data<-20000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

 return g_data ;
}

//-----------------------------------------
//int16_t Checkout_PGyr(int16_t g_data)
//------------------------------------------
int16_t Checkout_RGyr(int16_t g_data)
{
 static int First=1;
 static int g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>20000)
     g_data=g_data_Back;
   else if(g_data<-20000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

 return g_data ;
}
//-----------------------------------------
//int16_t Checkout_PGyr(int16_t g_data)
//------------------------------------------
int16_t Checkout_YGyr(int16_t g_data)
{
 static int First=1;
 static int g_data_Back;

 if(First)
 {
  First=0;
 }
 else
 {
   if(g_data>20000)
     g_data=g_data_Back;
   else if(g_data<-20000)
     g_data=g_data_Back;  
 }

  g_data_Back=g_data;

 return g_data ;
}

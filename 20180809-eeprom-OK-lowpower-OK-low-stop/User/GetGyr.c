#include "GetGyr.h"
#include "GetAcc.h"
#include "spi.h"
#include "Delay.h"
#include "commdata.h"



void CoordChangeG(void)
{
	int16_t  Gyr_XT,Gyr_YT,Gyr_ZT;			      //������X������
	
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
	
	 if(IMU_Kind==9)    //ģ��
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
	 else if(IMU_Kind==6)    //X�ᳬ��
	 {
	  Gyr_Y= -Gyr_XT;
		Gyr_X=  Gyr_YT;
		Gyr_Z=  Gyr_ZT;
	 }
	 else //if(IMU_Kind==7)    //X�ᳬǰ--��׼��װ
	 {
	  Gyr_X=  Gyr_XT;
		Gyr_Y=  Gyr_YT;
		Gyr_Z=  Gyr_ZT;	 
	 }
	}	 
	
	if(ANG_Kind_Flag==0)       //�������Ӧ����
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
		SPI_ADS_SendData(0x8c);//ע����ַ0x0c��ȡ0x8c
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



//void BMI055G_Ini(void)		 //������Ϊ+/-250dps,ODR=2KHz
//{
//		uint8_t GyrGet,GyrSet=0x03;
//	
//		DISABLE_BMI055G_SPI;	 
//		DISABLE_BMI055A_SPI;
//	  delay_us(30);	
//	
//		ENABLE_BMI055G_SPI;	     //CSB1_Acc-->PB5����͵�ƽ����ƬѡBMX055_Acc;
//		delay_us(5); 
//		SPI_ADS_SendData(0x14); //д��ַΪ0x14�ļĴ�����BGW_SOFTRESET��
//		SPI_ADS_SendData(0xB6); //�Դ��������������λ�Իָ�Ĭ�����á�����׼ģʽ��P100��;SPI�ӿڲ���4����(P113)������2000dps(P99)
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;     //data_high_bw=0�������ٶ�����Ϊ�˲������ݣ�SPIΪ8λģʽʱshadow_dis=1�������ܱ�֤����������-->����λ������������
//		 		
//		do
//		{
//			delay_ms(3);
//			ENABLE_BMI055G_SPI;			             //ƬѡBMX055_Gyr
//			delay_us(30); 
//			SPI_ADS_SendData(0x0F);              //д��ַΪ0x0f�ļĴ�����RANGE��
//			SPI_ADS_SendData(0x03);              //0-2000g;1-1000g,2-500g,3-250g;4-125g  
//			delay_us(50);	
//			DISABLE_BMI055G_SPI;	
//			
//			delay_ms(1);
//			
//			ENABLE_BMI055G_SPI;	                  //ƬѡBMX055_Acc
//			SPI_ADS_SendData(0x8F);               //����ַΪ0x02�ļĴ�����ACCD_X_LSB��
//			GyrGet =SPI_ADS_SendData(0x00);       //���Ĵ���ACCD_X_LSB			
//			DISABLE_BMI055G_SPI;
//		
//	  }while(GyrGet!=GyrSet);	
//	

//  	delay_us(5);		
//		ENABLE_BMI055G_SPI;
//		delay_us(30); 
//   	SPI_ADS_SendData(0x13);//д��ַΪ0x13�ļĴ�����RATE_HBW��
//		SPI_ADS_SendData(0x00);//���ٶ�����Ϊ�˲�������ݣ���֤����������
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;

//  	delay_us(5);
//		ENABLE_BMI055G_SPI;
//		delay_us(30); 
//		SPI_ADS_SendData(0x10);//д��ַΪ0x10�ļĴ�����BW��
//		SPI_ADS_SendData(0x05);//�˲�����Ϊ12HZ����С  P100
//		delay_us(50);	
//		DISABLE_BMI055G_SPI;

//}

//void  Get_BMI055G_Data(void)
//{
//		DISABLE_BMI055A_SPI;	 
//		DISABLE_BMI055G_SPI;
//    delay_us(100);	
//		 
//		ENABLE_BMI055G_SPI;                   //ƬѡBMX055_Gyr	
//		SPI_ADS_SendData(0x82);               //����ַΪ0x02�ļĴ�����RATE_X_LSB��

//		BMI055G_Rx[0] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_X_LSB
//		BMI055G_Rx[1] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_X_MSB

//		BMI055G_Rx[2] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_Y_LSB
//		BMI055G_Rx[3] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_Y_MSB

//		BMI055G_Rx[4] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_Z_LSB
//		BMI055G_Rx[5] =SPI_ADS_SendData(0x00);//���Ĵ���RATE_Z_MSB

//		DISABLE_BMI055G_SPI;

//    CoordChangeG();
//	 

//}

void Get_Rate_Data(void)
{


 floPitchGyr = ConvertorGyrData(Gyr_Y,GyrBias_Y,GyrScale_Y,0);   //ȥ��ƫ�ú�Ľ��ٶ�
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
//���ٶ��ź�ת��
//����ʲô���̣���תΪxx��/��
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

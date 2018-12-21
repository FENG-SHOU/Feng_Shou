#include "GetIMU.h"
 
 
//------------------------------
//void Send_IMUData(void)
//------------------------------
void Send_IMUData(void)
{
  static uint8_t i=0,Run_Flag;
	
	if(Debug_Flag)
	{	
	 Run_Flag=GNSS_Send_OK&&(SendTime>=DetaTime)&&Debug_Flag;	 
	}
	else
	{
	 if(User_Kind==2)
	 {
		 if(ZeroSecondFlag)
		 {			 
		  Run_Flag=GNSS_Send_OK&&(SendTime>=DetaTime);	
		 }
		 else
		 {
		  Run_Flag=SendTime>=DetaTime;	
		 }	 
	 }
	 else if(User_Kind==6)
	 {
		 if(ZeroSecondFlag)
		 {			 
		  Run_Flag=GNSS_Send_OK&&(SendTime>=DetaTime);	
		 }
	 }
	 else
	 {
		 Run_Flag=GNSS_Send_OK&&(SendTime>=DetaTime)&&Debug_Flag;		   	 
	 }
  }
	

	 
	 if(Run_Flag)   //200mһ������  ���Ϸ�������....
	 {	
     Run_Flag=0;		 
		 SendTime       = 0;                                //
		 ZeroSecondFlag =0;
		 
	
		 
		if(Gps_Fren_Kind==5)                                //���5Z����....������
		{
      GNSS_Send_OK   = 0;	                             //1HZû���������źţ��������ñ�־λ�����޷����½���ó���........
    }
		
		if((User_Kind==6)&&(Debug_Flag==0))
		{
		 DMA_Tx_Num=55;
		 IMU_DMA_Num = 0; 
		}
    else
		{			
		DMA_Tx_Num     = IMU_DMA_Num;
		IMU_DMA_Num = 0; 		                                //1hzû���������źţ��˹���㣬��洢���ݴ���Ϊ���µ�10������
		}
		
		if(DMA_Tx_Num>sizeof(m_IMUData))
			DMA_Tx_Num=sizeof(m_IMUData)-1;
		
 		 for(i=0;i<DMA_Tx_Num;i++)  
		 {
      m_IMUDataB[i] = m_IMUData[i];
     }

		 if(DMA_Tx_Num>0)
    {  	
			DMA_Send_Kind = 2;
		  USART_DMA_TX_Configuration(m_IMUDataB);
			SendDriver();
    }	 

  


	 }
}


//----------------------------------
//void GetUTCTime(void)
//----------------------------------
void GetUTCTime(void)
{	
static uint8_t CFlag=0;
	
	if(IMDMStime>=(1000/BaseTime))   //����1s����0�����¿�ʼ
	{
	 IMDMStime=0;
	 CFlag=1;	
	}	

	if(CFlag)     //û��GPS����ʱ��    //����GPS�����ʱ����GPS����Ѹñ������㣬�Ӷ�����1S...
	{
		CFlag=0;
		
		 IMDSecond++;
		if(IMDSecond==60)
		{
			IMDSecond=0;
			IMDMinute++;
			if(IMDMinute==60)
			{
				IMDMinute=0;
				IMDHour++;
				if(IMDHour==24)
				{
					IMDHour=0;
				}
			}
		}
	}	
}


void IMUData_Deal_New(uint8_t IGetData,uint8_t fisttime)
{
  uint8_t CheckNumData=0;
	int32_t Gyr_Temp=0;
	uint8_t  i=0;
  uint8_t  Imu_Up_Num=0;
	
	//----------------------------
	//����IMU_SData����Խ��....
	//-----------------------------
	if(IGetData>=45)   
		IGetData=45;
  //-----------------------------
	
	//----------------------------
	//����DAT_SData
	//----------------------------
	if(fisttime>=19)
		fisttime=19;
	//----------------------------
	
/*-----------------IMU֡ͷ--------------------------*/
  IMU_DATAT[0] = '$';
  IMU_DATAT[1] = 'G';
  IMU_DATAT[2] = 'P';
  IMU_DATAT[3] = 'I';
  IMU_DATAT[4] = 'M';
  IMU_DATAT[5] = 'U';
	IMU_DATAT[6] = ',';
	
/*-------------------Time����---------------------*/	
		
  if(((User_Kind==2)&&(Debug_Flag==0))||((User_Kind==6)&&(Debug_Flag==0)))
	{
	ProcessITOAH(DAT_SData[fisttime][0]);
  IMU_DATAT[7]  = Change_Data[0];    //0
  IMU_DATAT[8]  = Change_Data[1];    //1

  ProcessITOAH(DAT_SData[fisttime][1]);
  IMU_DATAT[9]  = Change_Data[0];	   //2
  IMU_DATAT[10]  = Change_Data[1];	 //3
 
  ProcessITOAH(DAT_SData[fisttime][2]);
  IMU_DATAT[11]  = Change_Data[0];	   //4
  IMU_DATAT[12]  = Change_Data[1];	   //5

  IMU_DATAT[13]  = '.';	             //6

  ProcessITOAM(DAT_SData[fisttime][3]);
  IMU_DATAT[14]  = Change_Data[0];	   //7
  IMU_DATAT[15]  = Change_Data[1];	   //8
  IMU_DATAT[16]  = Change_Data[2];	   //8
	IMU_DATAT[17]  = ',';	
	
	Imu_Up_Num=11;	
	}
  else
	{
	 Imu_Up_Num=0;
	}		
	//-----------------------------------------------
	//����
	//-----------------------------------------------
	if(Imu_Up_Num>=11)
		Imu_Up_Num=11;
	//-----------------------------------------------

		
/*-------------------����������---------------------*/	
	Gyr_Temp=IMU_SData[IGetData][0];

  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+7]  = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+8]  = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+9]  = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+10] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+11] = ',';
//---------------------------------------------

  Gyr_Temp=IMU_SData[IGetData][1];
	
	Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+12] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+13] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+14] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+15] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+16] = ',';	
	
//---------------------------------------------
	Gyr_Temp=IMU_SData[IGetData][2];
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+17] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+18] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+19] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+20] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+21] = ',';
/*-------------------����������---------------------*/
//---------------------------------------------

	Gyr_Temp=IMU_SData[IGetData][3];
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+22] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+23] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+24] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+25] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+26] = ',';
//---------------------------------------------
	Gyr_Temp=IMU_SData[IGetData][4];
		
	Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+27] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+28] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+29] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+30] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+31] = ',';	

	//---------------------------------------------
	Gyr_Temp=IMU_SData[IGetData][5];
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+32] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+33] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+34] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+35] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+36] = ',';
/*-------------------ʱ�����к�---------------------*/
  Change_Time(IGetData);
	IMU_DATAT[Imu_Up_Num+37] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+38] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+39] = '*';
/*-------------------У����-------------------------*/
  CheckNumData    = ProcessCheckIMU(Imu_Up_Num+40);
  ProcessCheckToAIMU(CheckNumData);
	IMU_DATAT[Imu_Up_Num+40] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+41] = Change_Data[1];
/*-------------------�س�����-----------------------*/	
  IMU_DATAT[Imu_Up_Num+42] = 0x0D;	          
  IMU_DATAT[Imu_Up_Num+43] = 0x0A;	
	
	
}


void IMUData_Deal(uint8_t IGetData,uint8_t fisttime)
{
  uint8_t CheckNumData=0;
	int32_t Gyr_Temp=0;
	uint8_t  i=0;
  uint8_t  Imu_Up_Num=0;
	
	//----------------------------
	//����IMU_SData����Խ��....
	//-----------------------------
	if(IGetData>=45)   
		IGetData=45;
  //-----------------------------
	
	//----------------------------
	//����DAT_SData
	//----------------------------
	if(fisttime>=19)
		fisttime=19;
	//----------------------------
	
/*-----------------IMU֡ͷ--------------------------*/
  IMU_DATAT[0] = '$';
  IMU_DATAT[1] = 'G';
  IMU_DATAT[2] = 'P';
  IMU_DATAT[3] = 'I';
  IMU_DATAT[4] = 'M';
  IMU_DATAT[5] = 'U';
	IMU_DATAT[6] = ',';
	
/*-------------------Time����---------------------*/	
		
  if(((User_Kind==2)&&(Debug_Flag==0))||((User_Kind==6)&&(Debug_Flag==0)))
	{
	ProcessITOAH(DAT_SData[fisttime][0]);
  IMU_DATAT[7]  = Change_Data[0];    //0
  IMU_DATAT[8]  = Change_Data[1];    //1

  ProcessITOAH(DAT_SData[fisttime][1]);
  IMU_DATAT[9]  = Change_Data[0];	   //2
  IMU_DATAT[10]  = Change_Data[1];	 //3
 
  ProcessITOAH(DAT_SData[fisttime][2]);
  IMU_DATAT[11]  = Change_Data[0];	   //4
  IMU_DATAT[12]  = Change_Data[1];	   //5

  IMU_DATAT[13]  = '.';	             //6

  ProcessITOAM(DAT_SData[fisttime][3]);
  IMU_DATAT[14]  = Change_Data[0];	   //7
  IMU_DATAT[15]  = Change_Data[1];	   //8
  IMU_DATAT[16]  = Change_Data[2];	   //8
	IMU_DATAT[17]  = ',';	
	
	Imu_Up_Num=11;	
	}
  else
	{
	 Imu_Up_Num=0;
	}		
	//-----------------------------------------------
	//����
	//-----------------------------------------------
	if(Imu_Up_Num>=11)
		Imu_Up_Num=11;
	//-----------------------------------------------
/*-------------------����������---------------------*/	
	Gyr_Temp=0;

  for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][0];
  }	 

	Gyr_Temp=Gyr_Temp/5;

  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+7]  = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+8]  = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+9]  = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+10] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+11] = ',';
//---------------------------------------------
	Gyr_Temp=0;
	for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][1];
  }	  
  Gyr_Temp=Gyr_Temp/5;
	
	Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+12] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+13] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+14] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+15] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+16] = ',';	
	
//---------------------------------------------
	Gyr_Temp=0;
	
  for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][2];
  }	 

	Gyr_Temp=Gyr_Temp/5;
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+17] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+18] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+19] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+20] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+21] = ',';
/*-------------------����������---------------------*/
//---------------------------------------------
	Gyr_Temp=0;
	
	for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][3];
  }	 
	Gyr_Temp=Gyr_Temp/5;
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+22] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+23] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+24] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+25] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+26] = ',';
//---------------------------------------------
	Gyr_Temp=0;
	for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][4];
  }	 
	Gyr_Temp=Gyr_Temp/5;
		
	Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+27] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+28] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+29] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+30] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+31] = ',';	

	//---------------------------------------------
	Gyr_Temp=0;
	for(i=0;i<5;i++)
  {
    Gyr_Temp+=IMU_SData[IGetData+i][5];
  }	 
	Gyr_Temp=Gyr_Temp/5;
	
  Change_SensorData(Gyr_Temp);
	IMU_DATAT[Imu_Up_Num+32] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+33] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+34] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+35] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+36] = ',';
/*-------------------ʱ�����к�---------------------*/
  Change_Time(IGetData);
	IMU_DATAT[Imu_Up_Num+37] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+38] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+39] = '*';
/*-------------------У����-------------------------*/
	
	CheckNumData    = ProcessCheckIMU(Imu_Up_Num+40);
  ProcessCheckToAIMU(CheckNumData);
	IMU_DATAT[Imu_Up_Num+40] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+41] = Change_Data[1];
/*-------------------�س�����-----------------------*/	
  IMU_DATAT[Imu_Up_Num+42] = 0x0D;	          
  IMU_DATAT[Imu_Up_Num+43] = 0x0A;	


}

//-------------------------------------------
//void Change_SensorData(uint16_t SensorData)
//-------------------------------------------
void Change_SensorData(uint16_t SensorData)
{
  static   uint8_t  i;
	Change_Data[0]=SensorData/16/16/16;
	Change_Data[1]=SensorData/16/16-Change_Data[0]*16;
	Change_Data[2]=SensorData/16-Change_Data[0]*16*16-Change_Data[1]*16;
	Change_Data[3]=SensorData-Change_Data[0]*16*16*16-Change_Data[1]*16*16-Change_Data[2]*16;
	
	for(i=0;i<4;i++)
	{
		if(Change_Data[i] <= 9)
		{
		Change_Data[i] = (Change_Data[i] + '0') ;
		}
		else
		{
		Change_Data[i] = ((Change_Data[i]-10) + 'A') ;
		}   
	}
}

//-----------------------------------
//void Change_Time(uint8_t SensorData)
//-----------------------------------
void Change_Time(uint8_t SensorData)
{
  static   uint8_t  i;
	Change_Data[0]=SensorData/16;
	Change_Data[1]=SensorData-Change_Data[0]*16;
	
	for(i=0;i<2;i++)
	{
		if(Change_Data[i] <= 9)
		{
		Change_Data[i] = (Change_Data[i] + '0') ;
		}
		else
		{
		Change_Data[i] = ((Change_Data[i]-10) + 'A') ;
		}   
	}
}


//-----------------------------------------
//uint8_t ProcessCheckIMU(uint8_t DNum)
//-----------------------------------------
uint8_t ProcessCheckIMU(uint8_t DNum)
{
	static  uint8_t i;
	static uint8_t CheckNum;

	CheckNum=0;
	for(i=1;i<DNum;i++)
	{
	CheckNum^=IMU_DATAT[i]; 
	}   
	return CheckNum;
}

//----------------------------------------
//void ProcessCheckToA(uint8_t CheckNum)
//----------------------------------------
void ProcessCheckToAIMU(uint8_t CheckNum)
{									
static   uint8_t  i;
  			     
   Change_Data[0] = CheckNum/16 ; 			       //ʮλ
   Change_Data[1] = CheckNum -Change_Data[0]*16;	   //��λ

   for(i=0;i<2;i++)
   {
    if(Change_Data[i] <= 9)
	{
	 Change_Data[i] = (Change_Data[i] + '0') ;
	}
	else
	{
	 Change_Data[i] = ((Change_Data[i]-10) + 'A') ;
	}   
   }
}



void IMDAT_Deal(uint8_t fisttime)
{
	IMUTC_Data[0]='$';
	IMUTC_Data[1]='I';
	IMUTC_Data[2]='5';
	IMUTC_Data[3]='D';
	IMUTC_Data[4]='A';
	IMUTC_Data[5]='T';
	IMUTC_Data[6]=',';
	
	//if(fisttime>=sizeof(DAT_SData))    //����DAT_SData[]�������ԣ���Ϊ�����һ����λ����
	//	fisttime=sizeof(DAT_SData)-1;
	
	if(fisttime>=19)    
	   fisttime=19;
//----------1-----9----------------
//UTCʱ��
//--------------------------------

  ProcessITOAH(DAT_SData[fisttime][0]);
  IMUTC_Data[7]  = Change_Data[0];    //0
  IMUTC_Data[8]  = Change_Data[1];    //1

  ProcessITOAH(DAT_SData[fisttime][1]);
  IMUTC_Data[9]  = Change_Data[0];	   //2
  IMUTC_Data[10]  = Change_Data[1];	 //3
 
  ProcessITOAH(DAT_SData[fisttime][2]);
  IMUTC_Data[11]  = Change_Data[0];	   //4
  IMUTC_Data[12]  = Change_Data[1];	   //5

  IMUTC_Data[13]  = '.';	             //6

  ProcessITOAM(DAT_SData[fisttime][3]);
  IMUTC_Data[14]  = Change_Data[0];	   //7
  IMUTC_Data[15]  = Change_Data[1];	   //8
  IMUTC_Data[16]  = Change_Data[2];	   //8
	
/*-------------------????-----------------------*/	
  IMUTC_Data[17] = 0x0D;	          
  IMUTC_Data[18] = 0x0A;	
}


void ProcessITOAH(uint8_t IMDATData)
{									
 static  uint8_t  i=0;
  			     
   Change_Data[0] = IMDATData/10 ;				   //??
   Change_Data[1] = IMDATData -Change_Data[0]*10;	   //??

   for(i=0;i<2;i++)
   {
    Change_Data[i] = Change_Data[i] + 0x30;   
   }
}

void ProcessITOAM(uint16_t IMDATData)
{									
 static  uint8_t  i;
  			     
   Change_Data[0] = IMDATData/100;				   //��λ
   Change_Data[1] = IMDATData/10-Change_Data[0]*10;	   //ʮλ
   Change_Data[2] = IMDATData-Change_Data[0]*100-Change_Data[1]*10;	 //ʮλ
   for(i=0;i<3;i++)
   {
    Change_Data[i] = Change_Data[i] + 0x30;   
   }
}




//--------------------------------
//void Save_IMUData(void)
//--------------------------------
void Save_IMUData(void)
{	
	uint16_t SaveNum;
	
	if(SaveTime>=DetaTime)   //1hz->50   5hz->10;  ��Ϊ�ú�������λ��..�ſ�����������..��Ϊ��Ҫ����Pack_IMU�� SaveTime PackTime�ı߽�����...Savetime==50 45
	SaveTime=0;
	
	if(IMU_Num>=DetaTime)    //1hz->50   5hz->10;        
	{
	  IMU_Num=0;
	}	
	
	//--------------------------
	  SaveNum=SaveTime;           //5HZ,200ms,10msһ��������ÿ��20��������0-19
	if(SaveNum>=DetaTime)
		SaveNum=DetaTime;
  //--------------------------
	
   IMU_SData[SaveNum][0] = Gyr_X;
	 IMU_SData[SaveNum][1] = Gyr_Y;
	 IMU_SData[SaveNum][2] = Gyr_Z;
	 IMU_SData[SaveNum][3] = intPitchAcc;
	 IMU_SData[SaveNum][4] = intRollAcc;
	 IMU_SData[SaveNum][5] = intAliAcc;
	 IMU_SData[SaveNum][6] = IMU_Num;
	 IMU_SData[SaveNum][7] = IMDMStime*BaseTime;		
	
}


//------------------------------------
//void Save_DATData(void)
//------------------------------------
void Save_DATData(void)
{	
	uint16_t SaveNum;
	
	if(SaveTime==0)   //0..... �ɼ�����ʱ��....
	{	
	 DAT_SData[0][0]=IMDHour;
	 DAT_SData[0][1]=IMDMinute;
	 DAT_SData[0][2]=IMDSecond;
	 DAT_SData[0][3]=IMDMStime*BaseTime;			
	}
	else
	{ 		
	 if(SaveTime>=DetaTime)                //��������...
	 {
     SaveTime=0;
   }		
	 
   if((SaveTime+1)%5==0)   //4->5,9->10,14->15.....44->45...49->50.. �ɼ�����ʱ��....���ԣ�11������....
   {
	 	//--------------------------
	    SaveNum=(SaveTime+1)/5;
	  if(SaveNum>=19)
		   SaveNum=19;
    //--------------------------
    DAT_SData[SaveNum][0]=IMDHour;
	  DAT_SData[SaveNum][1]=IMDMinute;
	  DAT_SData[SaveNum][2]=IMDSecond;
	  DAT_SData[SaveNum][3]=(IMDMStime+1)*BaseTime;			
   }
	 
  }	
}


//--------------------------------
//void IMU_ALL_DMA(void)
//--------------------------------
void IMU_ALL_DMA(void)
{	
		static uint8_t i;
    uint16_t Num=0;
   
 if(((User_Kind==2)&&(Debug_Flag==0))||((User_Kind==6)&&(Debug_Flag==0)))
 { 
		 
   for(i=0;i<55;i++)
		{
			Num=IMU_DMA_Num+i;
			
			//-------------------------------------
			if(Num>=sizeof(m_IMUData))
				Num=sizeof(m_IMUData)-1;
			//-------------------------------------	
			m_IMUData[Num]=IMU_DATAT[i];
		}   
		 
		IMU_DMA_Num+=55;
					
		if(Num>=sizeof(m_IMUData))
		Num=sizeof(m_IMUData)-1;
		
 }
 else
 {
		for(i=0;i<19;i++)
		{
			Num=IMU_DMA_Num+i;
			
			//-------------------------------------
			if(Num>=sizeof(m_IMUData))
				Num=sizeof(m_IMUData)-1;
			//-------------------------------------	
			
			m_IMUData[Num]=IMUTC_Data[i];
		}
		
		for(i=0;i<44;i++)
		{
			Num=IMU_DMA_Num+i+19;
			
			//-------------------------------------
			if(Num>=sizeof(m_IMUData))
				Num=sizeof(m_IMUData)-1;
			//-------------------------------------	
				
			m_IMUData[Num]=IMU_DATAT[i];
		}   
		 
		IMU_DMA_Num+=63;
					
		//-------------------------------------
		if(Num>=sizeof(m_IMUData))
		  Num=sizeof(m_IMUData)-1;
	  //-------------------------------------	
	}
}



//--------------------------
//void Pack_IMUData(void)
//--------------------------
void Save_ALLIMUData(void)
{
 if((User_Kind==2)||(User_Kind==6))
 {
  if((SaveTime<=DetaTime)&&(SendTime<=DetaTime))                   //200ms,10������һ�������ʵ���أ���
	{
	 if((SaveTime!=0)&&(SaveTime%5==0))     //5->0,10->5,15->10,20......45->40 50->45
	  {
	   IMDAT_Deal((SaveTime-5)/5);          //1-����UTCʱ������
	     
	   IMUData_Deal(SaveTime-5,(SaveTime-5)/5);	          //2-����IMU����
	
	   IMU_ALL_DMA();                       //3--DMA��������
   }
  }
 }
 else
 {
 	if((SaveTime<=DetaTime)&&(SendTime<=DetaTime)&&Debug_Flag)    //200ms,10������һ�������ʵ���أ���
	{
	 if((SaveTime!=0)&&(SaveTime%5==0))         //5->0,10->5,15->10,20......45->40 50->45
	  {
	   IMDAT_Deal((SaveTime-5)/5);              //1-����UTCʱ������
	     
	   IMUData_Deal(SaveTime-5,(SaveTime-5)/5);	//2-����IMU����
	
	   IMU_ALL_DMA();                           //3--DMA��������
   }
  }	
 }
}
//-------------------------------------------
//void  Get_BMI055_Data(void)
//-------------------------------------------
void  Get_BMI055_Data(void)//
{

	
	if(IMU_Flag)
	{
	IMU_Flag=0;         //0-��־λ��0
		
	//Get_BMI055G_Data();	//1-�������������
  Get_BMI160G_Data();
	Get_Rate_Data();    //1-��û��������ǵĽǶ���Ϣ��
  Get_Yaw_Data();

//	Get_BMI055A_Data();//2-��ü��ٶ�����
	Get_BMI160A_Data();
	Get_Acc_XYZ();	   //2-��ü��ٶ���ֵ��
	Get_Angle_XYZ();   //2-��û��ڼ��ٶȵ���̬��ֵ��	
	
  GetUTCTime();      //3-���UTCʱ��    ���û��gpsʱ�䣬��second++; 
	Save_DATData();    //3-�洢UTCʱ��		�洢5,10...�����ɼ�ʱ��.....
			
  Save_IMUData();	   //4-�洢IMU����    �洢ÿ֡....
//--------------------------------------------			
	SaveTime++;        //5-�洢�����ۼ�
	IMU_Num++;         //5-��Ŵ�������
	SendTime++;        //5-���ͱ�������
  IMDMStime++;       //5-����UTCʱ��		50    ->0
		
  //--------------------------------		
  if(SaveTime>=DetaTime+1)
	{	
	 GGA_Ini_Flag=1;
	}
  //-----------------------------	
	
	Save_ALLIMUData(); //6-���debug����洢10������.....5,10...���Time+Data......
 }
}

//-------------------------------
//void Pack_IMUData(void)
//-------------------------------
void Pack_IMUData(void)
{
  static int32_t IMU_SDatatT[6];
	static uint8_t i,j,PackFlag,D_Num;
	
	if(GpsDriverImuFlag)
	{
		if(PackTime==0)
		{
     PackFlag=1;               //��һ����������ȴ�.... 
    }
		else
		{
     PackFlag=GpsInsGetFlag;   //���������ݣ���Ҫ�ȴ�gpsins���   
    }
	 //--------------------------------------
	 //
	 //--------------------------------------		
	 if(PackFlag)
	 {
	  if((SaveTime-PackTime)>=5)   //1hz...Save=5,Pack=0(1)/Save=10,Pack=5(2).....Save=50,Pack=45(10)...
	  {                            //5hz...Save=5,Pack=0(1)/Save=10,Pack=5(2).....
		 PackTime = PackTime+5;      //0->5
		 	 
     GpsInsGetFlag = 0;
			   	
     for(i=0;i<6;i++)
		 {
      IMU_SDatatT[i]=0;
     }  
		
     for(i=0;i<6;i++)
		 {
			for(j=0;j<5;j++)
			{				
			 D_Num=PackTime-1-j;  //4,3,2,1,0     //9,8,7,6,5
				
			 if(D_Num>=20)
			 {
         D_Num=20;
       }
			 
			 if(D_Num<0)
				 D_Num=0;
			 
       IMU_SDatatT[i] += IMU_SData[D_Num][i];   //4-0=4;4-1=3;4-2=2;4-3=1;4-4=0;
			 
			}
    }
   
		IMUDataBuffer.Gyro[0][0]  = IMU_SDatatT[0]/5*GYRO_SCALE;  //�Ƕ�����
		IMUDataBuffer.Gyro[0][1]  = IMU_SDatatT[1]/5*GYRO_SCALE;
		IMUDataBuffer.Gyro[0][2]  = IMU_SDatatT[2]/5*GYRO_SCALE;
		
	  IMUDataBuffer.Acc[0][0]   = IMU_SDatatT[3]/5*ACC_SCALE;   //�ٶ�����
		IMUDataBuffer.Acc[0][1]   = IMU_SDatatT[4]/5*ACC_SCALE;
		IMUDataBuffer.Acc[0][2]   = IMU_SDatatT[5]/5*ACC_SCALE;
		
		IMUDataBuffer.MsrInterval  = INSUPDATE_SUBDATA_INTERVAL;
		
		IMUDataBuffer.UtcTime.Hour   = IMDHour;
		IMUDataBuffer.UtcTime.Minute = IMDMinute;
		IMUDataBuffer.UtcTime.Second = IMDSecond;
		

	  IMUDataBuffer.UtcTime.MillSecond = (IMU_SData[PackTime-5][7]);     //-5,�������ɼ�ʱ��....
		
	  //---------------------------------------
		if(PackTime>=DetaTime)         //��������һ��֮������....//1hz->50   5hz->10;
		{
			GpsDriverImuFlag =0;        //��Gps����Imu�����־��ʷʹ��
			PackTime         =0;
		}
    //---------------------------------------
		 if(PackDebug_Time)
		 {
		  PackDebug_Time=0;
			PackDebug_Flag=1;
		 }
		//---------------------------------------		
	  if(FrameFirst==1)             //����ǵ�һ֡������λ�ñ�־���ȴ�GNSSͬ��
		{
     IMUDataGetFlag  = TRUE;
		}
		else                          //������ǵ�һ֡�����õȴ���ֱ����λ
	  {
		 IMUDataReady    = TRUE;	  
    }
		//---------------------------------------
   }	 
  }
 }
}

//------------------------------
//void Data_Synch(void)
//------------------------------
void Data_Synch(void)
{
  	if(FrameFirst)
		{
		 if(GNSSDataGetFlag&&IMUDataGetFlag)  //���ͬ��...
		 {
		  FrameFirst=0;
			 
			GNSSDataGetFlag = FALSE;
			IMUDataGetFlag  = FALSE;

		  IMUDataReady    = TRUE;
			GNSSDataReady   = TRUE;			 
			 
			GpsDriverSendFlag  = 1;   //�������´��GPSԭʼ����.... 
	 	 }
		}
}


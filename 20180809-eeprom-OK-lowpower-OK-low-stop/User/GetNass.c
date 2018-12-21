#include "stm32f4xx.h"
#include "GetNass.h"
#include "BasicFunc.h"
#include "SendGPS.h"
#include "commdata.h"

//-------------------------
//void  Analyse_GpsData(void)
//-------------------------
void  Analyse_GpsData(void)
{
double temp0, temp1, temp2, temp3;
static uint8_t GGA_GGet_Flag=0,RMC_GGet_Flag=0,TimeNum,VTG_GGet_Flag=0,GST_GGet_Flag=0,Ins_Num=0,Gps_Num=0;
	uint8_t i;
	U32	SatIndex = 0;
	FLOAT32 MeanCn0=0.0;

 if(GpsDriverAnalyseFlag)
 {
	 GpsDriverAnalyseFlag=0;	 
 
 //---------------------------------
 //	
 //---------------------------------		
	 if(GGA_Get_Flag)
	{
    GGA_Get_Flag=0;
		
		IWDG_ReloadCounter();       //喂狗...
		
    GNSSDataBuffer.Position.Lon = GPGGAData.Longitude*DEG2RAD;
		GNSSDataBuffer.Position.Lat = GPGGAData.Latitude*DEG2RAD;
		GNSSDataBuffer.Position.Alt = GPGGAData.Altitude;
		
		GNSSDataBuffer.NavStatus   = GPGGAData.GPSQuality;
		
		GNSSDataBuffer.SatUseNum   = GPGGAData.NumOfSatsInUse;
		GNSSDataBuffer.Dops[1]     = GPGGAData.HDOP;
		
		if(GPGGAData.NumOfSatsInUse>7)
		{
     Diff_First_Flag=1;
    }
		
		GNSSDataBuffer.GetAgeFlag  = GetAgeDataFlag;
		
		GNSSDataBuffer.UtcTime.Hour       = GPGGAData.Hour;
		GNSSDataBuffer.UtcTime.Minute     = GPGGAData.Minute;
		GNSSDataBuffer.UtcTime.Second     = GPGGAData.Second;
		GNSSDataBuffer.UtcTime.MillSecond = GPGGAData.MSecond;
		
		if((ABS(GNSSDataBuffer.Position.Lon)>0)&&(ABS(GNSSDataBuffer.Position.Lat)>0))
		{
		 GNSSDataBuffer.NavFlag    |=0x03;                 //bit0-Lat Lon Valid, bit 1-Alt Valid
		}
		else
		{
     GNSSDataBuffer.NavFlag    =0x00;                 
    }

		if(DetaTime==50)
		{
     GNSSDataBuffer.Frenqucy=1;
    }
		else
		{
     GNSSDataBuffer.Frenqucy=5;
    }

		GetDiffAgeData();          
		
		GGA_GGet_Flag=1;
  }
		
	
 //---------------------------------
 //	               RMC
 //---------------------------------	
	if(RMC_Get_Flag)
	{
    RMC_Get_Flag=0;
		
		GNSSDataBuffer.UtcTime.Hour       = GPRMCData.Hour;
		GNSSDataBuffer.UtcTime.Minute     = GPRMCData.Minute;
		GNSSDataBuffer.UtcTime.Second     = GPRMCData.Second;
		GNSSDataBuffer.UtcTime.MillSecond = GPRMCData.MSecond;
		

		
			
		if((GNSSDataBuffer.UtcTime.Hour==0)&&(GNSSDataBuffer.UtcTime.Minute==0)&&(GNSSDataBuffer.UtcTime.Second==0))
		{
			Gps_Num++;
			if(Gps_Num>=5)
			{
			 Gps_Num=0;
			 ZeroSecondFlag=1;
			}
		}
		else
		{
				if((GNSSDataBuffer.UtcTime.Hour>0)||(GNSSDataBuffer.UtcTime.Minute>0)||(GNSSDataBuffer.UtcTime.Second>0))
				{
				 if(GNSSDataBuffer.UtcTime.MillSecond==0)
				 {
					ZeroSecondFlag=1;
				 }		
				}
		
		}
		
	  GNSSDataBuffer.Position.Lon = GPRMCData.Longitude*DEG2RAD;
		GNSSDataBuffer.Position.Lat = GPRMCData.Latitude*DEG2RAD;
		
		if((ABS(GNSSDataBuffer.Position.Lon)>0)&&(ABS(GNSSDataBuffer.Position.Lat)>0))
		{
		 GNSSDataBuffer.NavFlag    |=0x03;                 //bit0-Lat Lon Valid, bit 1-Alt Valid
		}
		else
		{
     GNSSDataBuffer.NavFlag    =0x00;                 
    }
		
		//---------------------------------
	  if(Ins_Go_Flag)
	  {
			Ins_Num++;
		  if(Ins_Num>=100)
		  {
		  	Ins_Num=100;
		    Ins_Go_Flag=0;
		  }
		  GPRMCData.CourseValid=1;
		  GPRMCData.Course=80;
		  GPRMCData.GroundSpeed=10;
	  }
	  else
	  {
	   Ins_Num=0;
	  }
		//---------------------------------
		
		
		 temp0 = GPRMCData.GroundSpeed*0.5144444444444;
		
		 if(GPRMCData.CourseValid==1)	            //对否？？？
		 {
		  temp1 = GPRMCData.Course*DEG2RAD;
			GNSSDataBuffer.RmcHeading=temp1;
		 }
		 else
		 {
		  temp1 = 0;
			GNSSDataBuffer.RmcHeading=0; 
		 }
		 
		 temp2 = sin(temp1);                                               
		 temp3 = cos(temp1);
		 GNSSDataBuffer.Velocity.Ve = temp0*temp2;  //东向速度
		 GNSSDataBuffer.Velocity.Vn = temp0*temp3;	//北向速度	
		 
		  if(GPRMCData.CourseValid==1)	            
		 {
		  GNSSDataBuffer.NavFlag |= 0x40;            //bit 6-Course Valid
		  GNSSDataBuffer.VelValid = 1;
		 }
		 else
		 {
		  GNSSDataBuffer.NavFlag &= 0xBF;            //bit 6-Course Valid
		  GNSSDataBuffer.VelValid = 0;
     }
		 
		  GNSSDataBuffer.NavFlag |= 0x10;            //bit 4-Level Velocity Valid
		//--------------------------------------------------

    GNSSDataBuffer.UtcTime.Year   = GPRMCData.Year;
		GNSSDataBuffer.UtcTime.Month  = GPRMCData.Month;
		GNSSDataBuffer.UtcTime.Day    = GPRMCData.Day;

		IMUDataBuffer.UtcTime.Year    = GPRMCData.Year;
		IMUDataBuffer.UtcTime.Month   = GPRMCData.Month;
		IMUDataBuffer.UtcTime.Day     = GPRMCData.Day;
	
		 //--------------------------------------------------
		if(GPRMCData.ModeIn=='N')
		{
     GNSSDataBuffer.NavType=0;
    }
    else if (GPRMCData.ModeIn=='A')	
		{
      GNSSDataBuffer.NavType=1;
    }
		else if(GPRMCData.ModeIn=='D')	
		{
     GNSSDataBuffer.NavType=2;
    }
		else if(GPRMCData.ModeIn=='E')	
		{
     GNSSDataBuffer.NavType=0xff;
    }
		
		//--------------------------------------------------
		// 如果无效...则.....
		//--------------------------------------------------
		if(GPRMCData.DataValid!='A')
		{
     GNSSDataBuffer.NavType   = 0;
		 GNSSDataBuffer.NavFlag   = 0;
		 GNSSDataBuffer.NavStatus = 0;
    }
		 

		RMC_GGet_Flag=1;
	}
 
	
	//---------------------------------
 //	              VTG
 //---------------------------------	
	if(VTG_Get_Flag)
	{
    VTG_Get_Flag=0;		
		
		temp0 = GPVTGData.speedNot*0.5144444444444;
		
		if(GPVTGData.CourseValid==1)	            //
		{
		  temp1 = GPVTGData.Ttrack*DEG2RAD;
			GNSSDataBuffer.RmcHeading=temp1;
		}
		else
		{
		  temp1 = 0;
			GNSSDataBuffer.RmcHeading=0; 
		}
		 
		temp2 = sin(temp1);                                               
		temp3 = cos(temp1);
		GNSSDataBuffer.Velocity.Ve = temp0*temp2;  //东向速度
		GNSSDataBuffer.Velocity.Vn = temp0*temp3;	//北向速度	
		 
    if(GPVTGData.CourseValid==1)	            
		{
		 GNSSDataBuffer.NavFlag |= 0x40;            //bit 6-Course Valid
		 GNSSDataBuffer.VelValid = 1;
		}
		else
		{
		 GNSSDataBuffer.NavFlag &= 0xBF;            //bit 6-Course Valid
		 GNSSDataBuffer.VelValid = 0;
    }
		 
		 GNSSDataBuffer.NavFlag |= 0x10;            //bit 4-Level Velocity Valid
		 
	 //--------------------------------------------------
		if(GPVTGData.PositMode=='N')
		{
     GNSSDataBuffer.NavType=0;
    }
    else if (GPVTGData.PositMode=='A')	
		{
      GNSSDataBuffer.NavType=1;
    }
		else if(GPVTGData.PositMode=='D')	
		{
     GNSSDataBuffer.NavType=2;
    }
		else if(GPVTGData.PositMode=='E')	
		{
     GNSSDataBuffer.NavType=0xff;
    }

		 VTG_GGet_Flag=1;
  }

	
	
	if(GST_Get_Flag)
	{
    GST_Get_Flag=0;
		
		GNSSDataBuffer.GstDetaLat = GPGSTData.GstDetaLat;
		GNSSDataBuffer.GstDetaLon = GPGSTData.GstDetaLon;
		GNSSDataBuffer.GstDeta    = sqrt(GPGSTData.GstDetaLon*GPGSTData.GstDetaLon+GPGSTData.GstDetaLat*GPGSTData.GstDetaLat);
		
		GST_GGet_Flag=1;
	}

	
	//---------------------------------
 //	
 //---------------------------------		
	if(GSA_Get_Flag)
	{
    GSA_Get_Flag=0;

	/*	for(i=0;i<(MAX_SVID/ 32 + 1);i++)
		{
     GNSSDataBuffer.SatUse[i]=GPGSAData.SatUse[i];
    }
		
		GNSSDataBuffer.Dops[0] = GPGSAData.PDOP;
		GNSSDataBuffer.Dops[1] = GPGSAData.HDOP;
		GNSSDataBuffer.Dops[2] = GPGSAData.VDOP;
		
		g_GsavInfo.Dops[0] = GPGSAData.PDOP;
		g_GsavInfo.Dops[1] = GPGSAData.HDOP;
		g_GsavInfo.Dops[2] = GPGSAData.VDOP;		
		
		g_GsavInfo.SatUseNum = GetBitNum(GNSSDataBuffer.SatUse, MAX_SVID / 32 + 1);
		
		GNSSDataBuffer.GSVAValid=1;*/
  }
  //---------------------------------
 //	
 //---------------------------------		
	if(GSV_Get_Flag)
	{
    GSV_Get_Flag=0;
		
	/*	for(i=0;i<(MAX_SVID/ 32 + 1);i++)
		{
     GNSSDataBuffer.SatInView[i]=GPGSVData.SatInView[i];
    }
		
		for(i=0;i<MAX_SVID;i++)
		{
     GNSSDataBuffer.SatCn0[i] = GPGSVData.SatCn0[i];
		 GNSSDataBuffer.SatEl[i]  = GPGSVData.SatEl[i];
		 GNSSDataBuffer.SatAz[i]  = GPGSVData.SatAz[i];
    }
		
		g_GsavInfo.SatInViewNum = GetBitNum(GNSSDataBuffer.SatInView, MAX_SVID / 32 + 1);
		g_GsavInfo.SatUseRate=(FLOAT32)g_GsavInfo.SatUseNum / (FLOAT32)g_GsavInfo.SatInViewNum;
		
		MeanCn0=0;
		SatIndex=0;
		while ((SatIndex = GetNextSvid(GNSSDataBuffer.SatUse, &SatIndex)) <= 0x1fff)
		MeanCn0 += (FLOAT32)GNSSDataBuffer.SatCn0[SatIndex];
	 
    if(g_GsavInfo.SatUseNum==0)
    g_GsavInfo.MeanCn0 = 0;
		else	
		g_GsavInfo.MeanCn0 = MeanCn0/ g_GsavInfo.SatUseNum;
				
		GNSSDataBuffer.GSVAValid=1;	*/
  }

	//------------------------------------------
	//
	//------------------------------------------
	if((GGA_GGet_Flag==1)&&((RMC_GGet_Flag==1)||(VTG_GGet_Flag==1)))      //此帧只要有GGA和(RMC或者VTG)就行
	{
		GGA_GGet_Flag=0;
		RMC_GGet_Flag=0;
		VTG_GGet_Flag=0;
		
	 
	//--------------------------------------------------
		GpsDriverImuFlag   = 1;   //驱动IMU_Pack...........

   if(Debug_Flag==0)
	 {
		 if(High_DGet_Flag==1)     //高频5HZ
		 {
       PackDebug_Time=1;	    //可以发送ATT+GSV.....
     }
		 else                      //低频1HZ
		 { 
		  if(GPGGAData.MSecond==0)
		  {	
       if(GPGGAData.Utc_Flag==0)
			 {
				TimeNum++; 
				if(TimeNum>=5)
				{
				 TimeNum=0;
				 PackDebug_Time=1;
				}
			 }
       else
       {	
	       PackDebug_Time=1;	    //可以发送ATT+GSV.....
			 }
		  }
	   }
	 }

	//--------------------------------------------------	
	 if(FrameFirst==0)      //如果不是第一帧，则不用等待...
	 {
	  GNSSDataReady   = TRUE;
   }
	 else                   //如果是第一帧，则等待IMU同步....
	 {
	  GNSSDataGetFlag  = TRUE;
   }		 
  }

 }
}
//-------------------------------------
//void GetDiffAgeData(void)
//-------------------------------------
void GetDiffAgeData(void)
{
 static U8 AgeFirst=1;
 static U8 AgeNum=0;
 static U8 AgeData=0;

 if(GetAgeDataFlag)    //如果发生过有效的差分龄期....
 {
  if(GINavResult.GpsHighFlag==0)  //如果低精度...
	{	
	//--------------------------------------------
   if(AgeFirst)
	 {
    AgeFirst = 0;
    GetAgeData = 10;
   }
	 
	 //--------------------------------------------
	 if(GNSSDataBuffer.Frenqucy==1)
	 {
	  GetAgeData = GetAgeData + 10;
	 }
	 else
	 {
	  GetAgeData = GetAgeData + 2;
   }
	 	 
	 //--------------------------------------------
	 if(GPGGAData.NumOfSatsInUse>0)  //如果有差分数据....则....
	 {      
	  if(InPutDiffNum>1)             //自动识别...
	  {
		  InPutDiffNum=0;
     GetAgeData = 6;
    } 	 
	 }	 
	 
	 if(GetAgeData>=600)
	 {
     GetAgeData = 600;
   }
	 
  }
	else
	{
    AgeFirst=1;
  }
 }
 else
 {
  GetAgeData = 600;
 }
 
}

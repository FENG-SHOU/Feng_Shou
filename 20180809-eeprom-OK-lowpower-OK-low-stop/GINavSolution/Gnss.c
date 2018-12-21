#include "Gnss.h"
#include "Const.h"
#include "Config.h"
#include "DataProc.h"
#include "BasicFunc.h"
#include "GlobalVars.h"
#include "SendGPS.h"
#include <string.h>
#include <math.h>





BOOL GnssEvaluation(PGNSS_DATA_T pGnssData,PIMU_DATA_T pImuData)
{

	static S32 i = 0;

	FLOAT32 Factor, delta_RIHeading, delta_PIHeading, delta_PRHeading, VLevel, VLevelB, Deta_Level;
	STATIC U16 Deta_SNum=0;
	static U16 Head_BNum,Head_SNum,GyrNum;
	

	STATIC POS_T    PosBack;
	STATIC VEL_T    VolBack;
	
	STATIC U8       PosFirst=0,MaxScale=1,BigErrorFlag=0;
	STATIC U16      Length_Big_Num=0,GpsBackNum=0;
	FLOAT32  Diff_North,Diff_East,Diff,GyrNumScale,GnssScale,DiffScale;
	
	
	//-----------------------------------------------------------------------	
	if (pGnssData->Dops[0] < 0.5f)
		pGnssData->Dops[0] = 99.0f;
	if (pGnssData->Dops[1] < 0.5f)
		pGnssData->Dops[1] = 99.0f;
	if (pGnssData->Dops[2] < 0.5f)
		pGnssData->Dops[2] = 99.0f;




	pGnssData->Sigma[0] = pGnssData->Sigma[1] = 6.25;     //水平速度噪声初值
	pGnssData->Sigma[2] = 9.0;                            //高度的噪声初值
	pGnssData->Sigma[3] = pGnssData->Sigma[4] = 0.05;     //水平速度噪声初值
	pGnssData->Sigma[5] = 0.10;                           //垂直速度噪声初值

	
	if(g_GINavInfo.GstStatus==4)                          //高精度之后，重启速度判断
	{
		g_GINavInfo.SpeedDFlag=1;
	}
	
	
	if(g_GINavInfo.GstStatus>=4)                           //高精度释放
	{
		Head_BNum = 0;                                       //高精度才释放.....
		
	}
	//----------------------------------------
	//           卫星导航飞点准备
	//----------------------------------------

	if(PosFirst==0)
	{
	  PosFirst = 1;
	  PosBack.Lat=pGnssData->Position.Lat;
	  PosBack.Lon=pGnssData->Position.Lon;
	  PosBack.Alt=pGnssData->Position.Alt;
	}
	//-------------------------------------------------------------
	//计算低速增益系数....
	//-------------------------------------------------------------
	VLevel   = sqrt(SQR(g_GINavInfo.Velocity.Ve) + SQR(g_GINavInfo.Velocity.Vn));
	VLevelB  = sqrt(SQR(pGnssData->Velocity.Ve) + SQR(pGnssData->Velocity.Vn));
	Deta_Level = ABS(VLevel-VLevelB);


    //-------------------------------------------------------------
	//  计算卫星质量增益系数
	//-------------------------------------------------------------
	GnssScale = (g_GINavInfo.GstDeta / g_GINavInfo.GstDetaMin) / g_GINavInfo.GstSScale;

	//-------------------------------------------------------------
     if (g_GINavInfo.GstStatus==4)
		  Factor = 1.0f;
	  else if (g_GINavInfo.GstStatus==5)
		  Factor = 20.0f*GnssScale*g_GINavInfo.GstBScale/g_GINavInfo.delta_HeadScale;                 
	  else if (g_GINavInfo.GstStatus==2)
		  Factor = 100.0f*GnssScale*g_GINavInfo.GstBScale/g_GINavInfo.delta_HeadScale;
	  else	   
	  {
      Factor = 1000.0f*GnssScale/g_GINavInfo.delta_HeadScale;  
	  }	  

	  for (i = 0; i < 3; i++)
		  pGnssData->Sigma[i] *= Factor;



	  if (g_GINavInfo.GstStatus==4)
		  Factor = 1.0f;
	  else if (g_GINavInfo.GstStatus==5)
		  Factor = 20.0f*GnssScale;                
	  else if (g_GINavInfo.GstStatus==2)
		  Factor = 200.0f*GnssScale; 
	  else	
	  {
		  Factor = 1000.0f * GnssScale;
	  }		

	  for (i = 3; i < 6; i++)
		  pGnssData->Sigma[i] *= Factor;  




	  if ((g_GINavInfo.GstSScale * 10) > 10.0)
	  {
		   BigErrorFlag = 0;
	  }
	  else
	  {
		  if (g_GINavInfo.Pos_Diff_Flag == 1)
		  {

			  BigErrorFlag = 1;
		  }
		  else
		  {
			  BigErrorFlag = 0;
		  }
	  }

	  //------------------------------------------
	  //       杠臂效应
	  //------------------------------------------



	 /* if(ABS(g_GINavInfo.Gyr_Rate)>2.0)
	  {
		  for (i = 3; i < 6; i++)
			  pGnssData->Sigma[i] *= ABS(g_GINavInfo.Gyr_Rate)*500;	
	  }
	  */

	  if(ABS(g_GINavInfo.Gyr_Rate)>5)
	  {
		  GyrNum++;

		  if((GyrNum>= pGnssData->Frenqucy*1)&&(GyrNum<= pGnssData->Frenqucy*10))   //1-10秒内....
		  {
			  for (i = 3; i < 6; i++)
				  pGnssData->Sigma[i] *= ABS(g_GINavInfo.Gyr_Rate)*500;	
		  }		 
	  }
	  else
	  {
		  GyrNum=0;
	  }
   
	  //--------------------------------------
	  // 如果一直转圈....则更加认可卫星轨迹
	  //--------------------------------------
	  if(GyrNum> pGnssData->Frenqucy*10)
	  {
		  if(GyrNum> pGnssData->Frenqucy*60)
		  {
			  GyrNum=300;
		  }

		  GyrNumScale=(GyrNum-45)/ pGnssData->Frenqucy;           //保证大于1

		  
		  Factor=1.0f/GyrNumScale;

		  for (i = 0; i < 3; i++)
			  pGnssData->Sigma[i] *= Factor;
	  }

	

	  if((pGnssData->SatUseNum< 4 )||(pGnssData->NavStatus==0)||(g_GINavInfo.GstStatus==0)||(g_GINavInfo.GST_Diff>g_GINavInfo.GstDetaMin/2))
	  {
		  g_GINavInfo.SpeedDFlag=0;
		  GpsBackNum=0;
		  return FALSE;	
	  }

	  
	  if((g_GINavInfo.GstBestNum>=g_GINavInfo.Frenquecy*10)&&(g_GINavInfo.KFCount>g_GINavInfo.Frenquecy*60*2))
	  {
		 for (i = 0; i < 6; i++)
			pGnssData->Sigma[i] *= 0.1;
	  }
	    
	  //-----------------------------------------
	  //Heading
	  //-----------------------------------------

   if((g_GINavInfo.StaticCount == 0) && ((VLevelB*10) > 3.0f )&&(g_GINavInfo.KFCount > g_GINavInfo.Frenquecy*20))
   {
	   
	   g_GINavInfo.Flag_Heading = 1;


	   //********************************************************************************
	   delta_RIHeading = (atan2(pGnssData->Velocity.Ve, pGnssData->Velocity.Vn) - g_GINavInfo.Euler.Phi)*RAD2DEG;
	
	   if (delta_RIHeading > 180.0f)
		   delta_RIHeading -= 360.0f;
	   if (delta_RIHeading < -180.0f)
		   delta_RIHeading += 360.0f;

	   delta_RIHeading = ABS(delta_RIHeading);

	   if (delta_RIHeading * 10>2.0f)
		{
		   delta_RIHeading = delta_RIHeading;
		}
		else
		{
			delta_RIHeading = 0.2f;
		}

	   g_GINavInfo.delta_RIHeading = delta_RIHeading;	  
	   Factor = delta_RIHeading;

	   //********************************************************************************
	   delta_PIHeading = (g_GINavInfo.Pos_Heading*DEG2RAD - g_GINavInfo.Euler.Phi)*RAD2DEG;

	   if (delta_PIHeading > 180.0f)
		   delta_PIHeading -= 360.0f;
	   if (delta_PIHeading < -180.0f)
		   delta_PIHeading += 360.0f;

	   delta_PIHeading = ABS(delta_PIHeading);

	   g_GINavInfo.delta_PIHeading = delta_PIHeading;


	   //********************************************************************************	

	   delta_PRHeading = (g_GINavInfo.Pos_Heading*DEG2RAD - atan2(pGnssData->Velocity.Ve, pGnssData->Velocity.Vn))*RAD2DEG;

	   if (delta_PRHeading > 180.0f)
		   delta_PRHeading -= 360.0f;
	   if (delta_PRHeading < -180.0f)
		   delta_PRHeading += 360.0f;

	   delta_PRHeading = ABS(delta_PRHeading);

	   g_GINavInfo.delta_PRHeading = delta_PRHeading;
	   
	   //********************************************************************************	


	   //-------------------------------------------------
	   //  低速倒车判断,非车库情况(非GST上升情况)
	   //--------------------------------------------------
	   if ((ABS(delta_RIHeading)>130) && (VLevelB<5.0) && (g_GINavInfo.GST_Diff<g_GINavInfo.GstDetaMin / 2))
	   {
		   GpsBackNum++;
		   if(GpsBackNum>= pGnssData->Frenqucy*4)
			 GpsBackNum= pGnssData->Frenqucy*4;
	   }
	   else
	   {
		   GpsBackNum=0; 
	   }

	   if(GpsBackNum>pGnssData->Frenqucy*2)   //超过2S认为在倒车....
	   {
		   g_GINavInfo.GpsHeadBack=1;
	   }
	   else
	   {
		   g_GINavInfo.GpsHeadBack=0;	
	   }
	   
	  //--------------------------------------------------------------------
	   if ((g_GINavInfo.GpsHeadBack == 0) && (g_GINavInfo.GST_Diff>g_GINavInfo.GstDetaMin / 5) && (BigErrorFlag == 0))
	   {
		   for (i = 0; i < 6; i++)
			   pGnssData->Sigma[i] *= 5 * delta_RIHeading;
	   }
	   else
	   {
		   //---------------------------------------------------------------------------------
		   //  如果前进，且方向误差时间很短，则
		   //---------------------------------------------------------------------------------
		   if ((g_GINavInfo.GpsHeadBack == 0) && (g_GINavInfo.GstBestNum < g_GINavInfo.Frenquecy * 20) && (BigErrorFlag == 0))
		   {
			   if (g_GINavInfo.GstStatus <= 2)
			   {
				   for (i = 0; i < 6; i++)
					   pGnssData->Sigma[i] *= 5 * delta_RIHeading;
			   }
			   else
			   {
				   for (i = 0; i < 6; i++)
					   pGnssData->Sigma[i] *= 5 * delta_RIHeading;
			   }
			  
		   }
	   }


	   g_GINavInfo.delta_RIHeading = delta_RIHeading;

	   if (ABS(g_GINavInfo.delta_RIHeading)<g_GINavInfo.DetaHead)
	   {
		   g_GINavInfo.delta_Num++;
	   }
	   else
	   {
		   g_GINavInfo.delta_Num = 0;
	   }


	   //--------------------------------------------------------------------
   }
   else
   {
	 GpsBackNum = 0;
     g_GINavInfo.delta_Num    = 0; 
	
	 g_GINavInfo.GpsHeadBack  = 0;	
	 g_GINavInfo.Flag_Heading = 0;
   }
  
  //-----------------------------------------------------------
  //    如果5秒之内，都在10度之后，说明ublox在平滑前进...
  //    计算轨迹平滑增益参数............
  //-----------------------------------------------------------

  if (g_GINavInfo.delta_Num>5 * 120)   //10s
  {
	  g_GINavInfo.delta_Num = 5 * 120;

  }

  if (g_GINavInfo.delta_Num>5 * 5)   //10s
  {
	  g_GINavInfo.delta_HeadScale = (g_GINavInfo.delta_Num - 20) / 5;

	  if (g_GINavInfo.GstStatus == 4)
	  {
		  MaxScale = 1;
	  }
	  else if (g_GINavInfo.GstStatus == 5)
	  {
		  MaxScale = 5;
	  }
	  else if (g_GINavInfo.GstStatus == 2)
	  {
		  MaxScale = 10;
	  }
	  else
	  {
		  MaxScale = 20;
	  }

	  if(g_GINavInfo.delta_HeadScale>MaxScale)
	  {
		  g_GINavInfo.delta_HeadScale=MaxScale;
	  }

   }
   else
   {
	  g_GINavInfo.delta_HeadScale=1.0;	 
   }



	 /* if(g_GINavInfo.KFCount > g_GINavInfo.Frenquecy*120)
	   {
			//----------------------------------------------------------------------------------
      //    如果GST上升很快......
	    //----------------------------------------------------------------------------------
		   if(g_GINavInfo.GST_Diff>g_GINavInfo.GstDetaMin)     //如果GST上升，则退出....
		   {
			   g_GINavInfo.SpeedDFlag=0;
			//   return FALSE;	
		   }

			 //----------------------------------------------------------------------------------
	     //   如果速度变化很快
	    //----------------------------------------------------------------------------------
		   if((VLevel>6)&&(Deta_Level>1)&&(g_GINavInfo.GstStatus!=4)&&(g_GINavInfo.SpeedDFlag)&&(Deta_SNum<=5*2)) //高速运行 //阈值1m/s....经过高精度,如果没有其他原因...可以触发该中断....该中断只能持续5秒
		   {
               Deta_SNum++;
			 //  return FALSE;	
		   }
		   else
		   {
               Deta_SNum=0;
		   }
	   }*/
	   



   
	//------------------------------------------
	//  如果速度超过最大值...
	//------------------------------------------

 	VLevel = sqrt(SQR(pGnssData->Velocity.Ve) + SQR(pGnssData->Velocity.Vn));
	VLevelB = sqrt(SQR(VolBack.Ve) + SQR(VolBack.Vn));


	if((ABS(VLevel-VLevelB)>3.0)&&(g_GINavInfo.GstStatus!=4))   
	{
		for (i = 3; i < 6; i++)
			pGnssData->Sigma[i] *= 800;	  
	}

	VolBack.Ve  = pGnssData->Velocity.Ve;
	VolBack.Vn  = pGnssData->Velocity.Vn;
	VolBack.Vu  = pGnssData->Velocity.Vu;


	if(VLevel>Max_Speed)
	{
		g_GINavInfo.SpeedDFlag=0;
		return FALSE;
	}

	//-----------------------------------------
	//  飞点
	//-----------------------------------------

	Diff_East    =  (pGnssData->Position.Lon*RAD2DEG - PosBack.Lon*RAD2DEG)*N*1000.0*cos(pGnssData->Position.Lat)/360.0;
	Diff_North   =  (pGnssData->Position.Lat*RAD2DEG - PosBack.Lat*RAD2DEG)*M*(1000.0/360.0);      	 

	Diff = sqrt(Diff_East*Diff_East+Diff_North*Diff_North);

	PosBack.Lat=pGnssData->Position.Lat;
	PosBack.Lon=pGnssData->Position.Lon;
	PosBack.Alt=pGnssData->Position.Alt;

	if(ABS(Diff)>Deta_Pos)
	{
		g_GINavInfo.SpeedDFlag=0;
		return  FALSE;
	}




	if ((ABS(g_GINavInfo.Pos_Diff)>3000) && (Length_Big_Num <= g_GINavInfo.Frenquecy * 60) && (g_GINavInfo.KFCount > 100))
	{
		Length_Big_Num++;
		if(Length_Big_Num>=g_GINavInfo.Frenquecy*120)
		Length_Big_Num=g_GINavInfo.Frenquecy*120;
		
		return  FALSE;
	}
	else
	{
	   Length_Big_Num=0;
	}
	

	return TRUE;
}

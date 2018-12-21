#include "GINavMain.h"
#include "GlobalVars.h"
#include "DataProc.h"
#include "InsAlign.h"
#include "InsNav.h"
#include "GIFilter.h"
#include "BasicFunc.h"
#include "SendGps.h"
#include <string.h>
#include <math.h>


void GINavInit(void)
{
	FLOAT64 sa, ca, sb, cb, sc, cc,yaw;
	
	
	YZ_Acce_Deta = 3.0;       //??--???--???
	YZ_Dece_Deta = -4.5;      //??--???--???

	YZ_Lane_Deta = 4.0;       //??--???--???
	YZ_Lane_Anlge = 20;       //??--??----?????

	YZ_Turn_Deta = 4.0;       //??--???--???
	YZ_Turn_Anlge = 45;       //??--??----?????

	YZ_Coll_Deta = 20;        //??--???--????

	YZ_Stab_Deta = 20;        //??--???--?????
	YZ_Stab_Time = 3;         //??--??----?????

	YZ_Att_Deta = 70;         //??--??----????

	YZ_Att_Min_Deta = 20;     //??--??----?????
	YZ_Att_Max_Deta = 70;     //??--??----?????
	
	
	if(g_GINavInfo.ResetFlag == 1)
	{
		g_GINavInfo.ResetFlag = 0;

		

		g_GINavInfo.Euler.Gamma = 0;
		g_GINavInfo.Euler.Phi = 0;
		g_GINavInfo.Euler.Theta = 0;

		Euler2Quat(&g_GINavInfo.Euler, &g_GINavInfo.Quat_bn);
		Quat2CM(&g_GINavInfo.Quat_bn, &g_GINavInfo.CM_bn);

		g_GINavInfo.INSState = INS_ACTIVE;
		g_GINavInfo.SYSFlag  = 0;
	  g_GINavInfo.GoodCount= 0;	

 
 //------------------------------------------------------------------------------------
 //�����û���ҵ����������ˣ�������1����ΪFlash�İ�װ�������⣬�������Զ�Ѱ�Ұ�װ��	
 //                          ������2�ǳ����¼λ��...�û������������������������Զ�Ѱ�Ұ�װ��
 //------------------------------------------------------------------------------------
		
		g_GINavInfo.InstallFlag = 1;		//�����Զ�Ѱ�Ұ�װ�ǵĹ�����.......	
		
		if(g_GINavInfo.Acc_D_Num<=1) 
		{
  		g_GINavInfo.Ini4_Flag=0;   //
			g_GINavInfo.Ini_Flag =1;   //�ǳ���Ҫ����
			
			RSE_DGet_Flag=0;          //�洢λ�÷���
			ANG_DGet_Flag=0;          //�洢�������		
			
			yaw = (0)*DEG2RAD;
			sa = sin(0);
			ca = cos(0);
			sb = sin(0);
			cb = cos(0);

			sc = sin(yaw);
			cc = cos(yaw);


			g_GINavInfo.Ini4_Mat.C11 = cc*ca;
			g_GINavInfo.Ini4_Mat.C12 = cc*sa*sb - sc*cb;
			g_GINavInfo.Ini4_Mat.C13 = cc*sa*cb + sc*sb;

			g_GINavInfo.Ini4_Mat.C21 = sc*ca;
			g_GINavInfo.Ini4_Mat.C22 = sc*sa*sb + cc*cb;
			g_GINavInfo.Ini4_Mat.C23 = sc*sa*cb - cc*sb;
			g_GINavInfo.Ini4_Mat.C31 = -sa;
			g_GINavInfo.Ini4_Mat.C32 = ca*sb;
			g_GINavInfo.Ini4_Mat.C33 = ca*cb;

		
		}
	//------------------------------------------------			
		GIKFInit();
	}
	else
	{

	MEMSET(&g_GINavInfo, 0, SIZEOF(GINAV_INFO_T));
	g_GINavInfo.INSState = INS_ACTIVE;

	g_GINavInfo.SYSFlag  = 0;


	g_GINavInfo.ImuCfg.InstallMat.C11 = g_GINavInfo.ImuCfg.InstallMat.C22 = g_GINavInfo.ImuCfg.InstallMat.C33 = 1.0;

	g_GINavInfo.Ini0_Mat.C11 = g_GINavInfo.Ini0_Mat.C22 = g_GINavInfo.Ini0_Mat.C33 = 1.0;
	g_GINavInfo.Ini1_Mat.C11 = g_GINavInfo.Ini1_Mat.C22 = g_GINavInfo.Ini1_Mat.C33 = 1.0;
	g_GINavInfo.Ini2_Mat.C11 = g_GINavInfo.Ini2_Mat.C22 = g_GINavInfo.Ini2_Mat.C33 = 1.0;
	g_GINavInfo.Ini3_Mat.C11 = g_GINavInfo.Ini3_Mat.C22 = g_GINavInfo.Ini3_Mat.C33 = 1.0;
	g_GINavInfo.Ini4_Mat.C11 = g_GINavInfo.Ini4_Mat.C22 = g_GINavInfo.Ini4_Mat.C33 = 1.0;


	g_GINavInfo.Ini0_Flag = 0;
	g_GINavInfo.Ini1_Flag = 0;
	g_GINavInfo.Ini2_Flag = 0;
	g_GINavInfo.Ini3_Flag = 0;
	g_GINavInfo.Ini4_Flag = 0;



  g_GINavInfo.GstFlag   = 0;
  g_GINavInfo.GstStatus = 0;
	g_GINavInfo.GstDeta   = 10;
	g_GINavInfo.GstDetaMin= 100.0;

	g_GINavInfo.Frenquecy = INS_UPDATE_RATE;
	g_GINavInfo.GyrDrift[0] = 0;
	g_GINavInfo.GyrDrift[1] = 0;
	g_GINavInfo.GyrDrift[2] = 0;
	g_GINavInfo.GST_Diff = 0.0;
	g_GINavInfo.DetaPitch = 0.0;
	g_GINavInfo.ImuCfg.InstallMatInitFlag = 0;

	g_GINavInfo.delta_HeadScale = 1.0;
	g_GINavInfo.GstBScale = 1.0;
	g_GINavInfo.GstScale = 1.0;

	g_GINavInfo.MaxSpeed = 27.8*1.2;

	g_GINavInfo.MaxAlti = 10.0;
	g_GINavInfo.MinAlti = 10.0;

		g_GINavInfo.ResetFlag = 0;
		g_GINavInfo.ResetFlagBack = 0;
		g_GINavInfo.ResetAlignFlag = 0;
		
		g_GINavInfo.Head_Flag   = 1;
		
		g_GINavInfo.Ini_Flag    = 0;		 
		g_GINavInfo.InstallFlag = 0;

		DataProcerInit();
		GIKFInit();
		

	}
	
}



















//---------------------------------------------
//BOOL GINavProc(POUTPUT_INFO_T pNavResult
//---------------------------------------------


BOOL GINavProc(POUTPUT_INFO_T pNavResult)
{
	S32 i, j;
	
	BOOL UpdateByGNSS=FALSE,UpdateByStatic=FALSE,UpdateByRMC=FALSE;	

	STATIC POS_T   PosBack;
	STATIC VEL_T   VelBack;
	STATIC EULER_T EulerBack;
	PGNSS_DATA_T   pGnssData;
	
	float AgnelTemp;

	STATIC U8    TimeNum=0,FirstFlag=1;
	STATIC U8    NavStatus,GNSS_Run_Flag,ZeroSecond;
  STATIC U16   GstBestNum=0,GstBadNum=0,AgeGetNum=0;
	FLOAT64 sa, ca, sb, cb, sc, cc,yaw;

//----------------------------------

	PIMU_DATA_T pImuData = GetIMUData();      //gty ��IMUDataBuffer���IMU����
	                                          //���IMU���£���pImuDataָ��IMU����
	                                          //���IMU���£���pImuDataָ��NULL
	if (!pImuData || g_GINavInfo.INSState == INS_INACTIVE)  
		return FALSE;


	pGnssData = GetGNSSData();                 //gty ��GNSSDataBuffer���GPS���� 
	                                           //���GNSS���£���pGnssDataָ��Gps����
											                       //���GNSSû�£���pGnssDataָ��Null  
	
	if(GNSS_Run_Flag==1)                       //���GNSS_Run_FlagΪ1���������һ��GNSSû�����....
	{
	 GNSS_Run_Flag=0;
	 return FALSE;
  }
	
	GNSS_Run_Flag=1;                           //����GNSS��ʼ....
	
//------------------------------------------------------------------
  if(pGnssData)
	{
		g_GINavInfo.NavStatus = pGnssData->NavStatus;
		g_GINavInfo.SatUseNum = pGnssData->SatUseNum;


		pGnssData->Frenqucy = 5;

		g_GINavInfo.GpsSpeed = sqrt(SQR(pGnssData->Velocity.Ve) + SQR(pGnssData->Velocity.Vn));
		//-----------------------------------------------
		// ���GST
		//-----------------------------------------------


		//---------------------------------------
	//      �ߵ;�������	
  //---------------------------------------		
	if(g_GINavInfo.GetAgeFlag)
	{
   AgeGetNum++;
  }

  if(AgeGetNum>g_GINavInfo.Frenquecy*30*3)
	{
    AgeGetNum = g_GINavInfo.Frenquecy*30*3;
  }		
	
	if(SGPS_DGet_Flag)            //������иߵ;����ж�
	{
		if((AgeGetNum>=g_GINavInfo.Frenquecy*5*1)&&(IS_INS_ALIGNED(g_GINavInfo.INSState)))  //5��֮��......
		{		
		 if((g_GINavInfo.GstDeta<g_GINavInfo.GstDetaMin*3)&&(g_GINavInfo.SatUseNum>=6))   
		 {
			 GstBestNum++;
			 GstBadNum=0;		 
		 }
		 else
		 {
			 GstBestNum=0;
			 GstBadNum++;		 
		 }	

		 if(GstBestNum>g_GINavInfo.Frenquecy*40)  //���80���ںõĻ�����.......
		 {
			GstBestNum  = g_GINavInfo.Frenquecy*40;
			g_GINavInfo.GpsHighFlag=1;
		 }
		 
		 if(GstBadNum>g_GINavInfo.Frenquecy*5)    //���10���Ӷ��ڻ��Ļ�����.......
		 {
			GstBadNum   = g_GINavInfo.Frenquecy*5;
			g_GINavInfo.GpsHighFlag=0;
		 }	 
		 
		} 
		else
		{  
		 g_GINavInfo.GpsHighFlag=1;  
		}
	}
	else
	{
	 g_GINavInfo.GpsHighFlag=5;
	}
	
	 
	 //----------------------------------------------
	 //
	 //----------------------------------------------
		
  if((pGnssData->UtcTime.Hour==0)&&(pGnssData->UtcTime.Minute==0)&&(pGnssData->UtcTime.Second==0))
		{
      TimeNum++; 
      if(TimeNum>=5)
		  {
       TimeNum=0;
			 ZeroSecond=1;
      }
			else
			{
			 ZeroSecond=0;
      }
    }       
		else
		{
	   if(pGnssData->UtcTime.MillSecond==0)
	   {
      ZeroSecond=1;
     }   
	   else
	   {
      ZeroSecond=0;
     }		
	  }
		
		//-----------------------------------------------
    //   ......����Ƕ���ֵ....
		//-----------------------------------------------

		NavStatus = g_GINavInfo.GstStatus;

		if (NavStatus == 4)
		{
			g_GINavInfo.GpsBadNum = 0;

			g_GINavInfo.DetaHead = 10;
		}
		else
		{
			g_GINavInfo.GpsBadNum++;
			if (g_GINavInfo.GpsBadNum>pGnssData->Frenqucy * 60 * 10)     //5*60=300 5����
			{
				g_GINavInfo.GpsBadNum = pGnssData->Frenqucy * 60 * 10;
			}
			g_GINavInfo.DetaHead = 3 + (FLOAT32)g_GINavInfo.GpsBadNum*1.0f / pGnssData->Frenqucy;   //

			if (g_GINavInfo.DetaHead>10.0f)
				g_GINavInfo.DetaHead = 10.0f;
		}
	}
	//------------------------------------------------------------------	

	
	//UTC��GPSʱ�����
	g_GINavInfo.Tag++;                        //gty �ñ���һֱ������....
	MEMCPY(&g_GINavInfo.UtcTime, &pImuData->UtcTime, sizeof(UTC_T));
	//GetGpsTime(&pImuData->UtcTime, &g_GINavInfo.GPSTime);             //gty ����GPS�ܺ�����...


	//���Ըտ���ʱ��0.5s����
	//	if (g_GINavInfo.Tag <= INS_UPDATE_RATE/2)             //gty�� ������g_GINavInfo.Tag����������Ӷ�С��INS_UPDATE_RATE/2
	//		goto END;



	//��̬���
	DynamicModeIdentify(pImuData);                         //����ԭ������ϵ�����жϣ���Ӱ���κ�Ч��

	//��ת����
	HandleCompensate(pImuData);                            //


  //-------------------------------------------------------------
  //�̶���װ��
  //-------------------------------------------------------------
	 if(ANG_Lock_Flag)
	 {		 
	  g_GINavInfo.Ini4_Flag = 1;	 
	 }
	 else
	 {
		//-------------------------------------------------------------
		//Ѱ�Ұ�װ��������.....
		//-------------------------------------------------------------
		 if(g_GINavInfo.InstallFlag==1)                           //
		 {	
			 //----------------------------------------		 
			 if((ANG_DGet_Flag==1)&&(g_GINavInfo.Ini_Flag==0))  //����а�װ�ǣ�������Ǹ�λ�������װ�������䣬��....
			 {
				g_GINavInfo.Install_Ini_Flag =2;
			 }
			 else
			 {
				g_GINavInfo.Install_Ini_Flag =1;
			 }
			 
			 //----------------------------------------
			 if(g_GINavInfo.Install_Ini_Flag==2)
			 {
				 if((ANG_DGet_Flag==1)&&(g_GINavInfo.Ini4_Flag==0))
				 {
				 //���ð�װ��			 
					g_GINavInfo.Acc_M_Angle  = MisAngleData;
					g_GINavInfo.Angle_M_Flag = 1;	

					yaw = (g_GINavInfo.Acc_M_Angle)*DEG2RAD;
					sa = sin(0);
					ca = cos(0);
					sb = sin(0);
					cb = cos(0);

					sc = sin(yaw);
					cc = cos(yaw);


					g_GINavInfo.Ini4_Mat.C11 = cc*ca;
					g_GINavInfo.Ini4_Mat.C12 = cc*sa*sb - sc*cb;
					g_GINavInfo.Ini4_Mat.C13 = cc*sa*cb + sc*sb;

					g_GINavInfo.Ini4_Mat.C21 = sc*ca;
					g_GINavInfo.Ini4_Mat.C22 = sc*sa*sb + cc*cb;
					g_GINavInfo.Ini4_Mat.C23 = sc*sa*cb - cc*sb;
					g_GINavInfo.Ini4_Mat.C31 = -sa;
					g_GINavInfo.Ini4_Mat.C32 = ca*sb;
					g_GINavInfo.Ini4_Mat.C33 = ca*cb;

					g_GINavInfo.Ini4_Flag = 1;			 
				}
			 }		 


			 
		 }
 }

	//IMU������Ϊ��ϵ�������
	IMUCompensate(pImuData);                               //gty IMU��װ�Ǿ��󲹳����������ƫ�ò���....

	


	if (!IS_INS_ALIGNED(g_GINavInfo.INSState))      //gty��    �����̬��λ���ٶȣ���������������ɳ�ʼ��...
	{	
			if (g_GINavInfo.ImuCfg.InstallMatInitFlag == 0)
			{
				InitInstallMat(pImuData);                           //���InstallMatInitFlag=1  ��ոճ�ʼ����װ����
			}
			else
			{
				g_GINavInfo.ImuCfg.InstallMatInitFlag <<= 1;      //gty ���InstallMatInitFlag=3������ΪIMU�������....
				g_GINavInfo.ImuCfg.InstallMatInitFlag |= 0x01;
				

		 
		  }	  
		
		
		
		//��ʼ������Bias
		InitGyroBias(pImuData, pGnssData);

		//INS��ʼ��׼
		INSAlign(pImuData, pGnssData);
		
		if (IS_INS_ALIGNED(g_GINavInfo.INSState))    //gty ��һ������...���ʼ��...
		{//��׼��ɣ���ʼ��Kalman
			GIKFInitPMatrix();
			g_GINavInfo.INSAloneMsCount = 200000;
			g_GINavInfo.KFCount = 0;
			g_GINavInfo.GNSSHaltCount = 0;
	
		}		
	}
	else
	{
		//INS����ȷ��
		if ((g_GINavInfo.INSState & INS_HEADING_GOOD) == 0)    //gty ��ʼ����ɺ���һ���ٶ������ڣ�gps����ǳ�ʼ��INS����....
			ConfirmHeading(pGnssData);

		g_GINavInfo.ResetFlag = 0;
		g_GINavInfo.ResetAlignFlag = 0;

	  if(g_GINavInfo.Acc_D_Num >= 20)
	  {	
		 g_GINavInfo.SYSFlag   = 4;
	  }
		else
		{
		 g_GINavInfo.SYSFlag   = 3;
		}
		//----------------------------------------
		//  λ�ñ���
		//----------------------------------------

		PosBack.Lat = g_GINavInfo.Position.Lat;
		PosBack.Lon = g_GINavInfo.Position.Lon;
		PosBack.Alt = g_GINavInfo.Position.Alt;

		VelBack.Ve = g_GINavInfo.Velocity.Ve;
		VelBack.Vn = g_GINavInfo.Velocity.Vn;
		VelBack.Vu = g_GINavInfo.Velocity.Vu;

		EulerBack.Gamma = g_GINavInfo.Euler.Gamma;
		EulerBack.Theta = g_GINavInfo.Euler.Theta;


		//��������

		INSUpdate(pImuData, TRUE, TRUE, TRUE);


		//����PHI��
		GIKFCalcPHIMatrix(pImuData->MsrInterval*INS_UPDATE_SAMPLE_NUM);

		//P��Ԥ��
		GIKFPredictPMatrix(pImuData->MsrInterval*INS_UPDATE_SAMPLE_NUM);

		if (g_GINavInfo.StaticCount == 0)
			g_GINavInfo.INSAloneMsCount += pImuData->MsrInterval*INS_UPDATE_SAMPLE_NUM;  //�����̬����INS��������ʱ������100ms,



		if (g_GINavInfo.INSAloneMsCount>500000)
			g_GINavInfo.INSAloneMsCount = 500000;

		g_GINavInfo.GNSSHaltCount++;


		UpdateByGNSS = !GIKFUpdateByGNSS(pGnssData, pImuData);


		//Kalman�������
		if (UpdateByGNSS || UpdateByRMC || UpdateByStatic )   //gty? �����һ��ʧ�ܵģ���INSAloneMsCount���㣬����λINSState.....
		{
			g_GINavInfo.KFCount = 0;
			g_GINavInfo.INSAloneMsCount = 0;
			g_GINavInfo.INSState = INS_ACTIVE;
			g_GINavInfo.ResetFlag = 1;
			g_GINavInfo.ResetFlagBack = 1;

		}

		

		
		

		//������������
		if (g_GINavInfo.INSAloneMsCount <= 10000 && g_GINavInfo.KFCount > 60 && (g_GINavInfo.INSState & INS_HEADING_GOOD) != 0)
			g_GINavInfo.PositionQuality = Excellent;
		else if (g_GINavInfo.INSAloneMsCount <= 100000 && g_GINavInfo.KFCount > 60 && (g_GINavInfo.INSState & INS_HEADING_GOOD) != 0)
			g_GINavInfo.PositionQuality = Good;
		else if (g_GINavInfo.INSAloneMsCount <= 800000)
			g_GINavInfo.PositionQuality = Bad;
		else
		{
			g_GINavInfo.PositionQuality = Unknow;
			//g_GINavInfo.INSState = INS_ACTIVE;
		}

	}

END:
	//������һ�����һ���������IMU���������´ν�������
	MEMCPY(g_GINavInfo.LastGyro, pImuData->Gyro[INS_UPDATE_SAMPLE_NUM - 1], sizeof(FLOAT64)* 3);
	MEMCPY(g_GINavInfo.LastAcc, pImuData->Acc[INS_UPDATE_SAMPLE_NUM - 1], sizeof(FLOAT64)* 3);
	
	//���������
	pNavResult->Tag = g_GINavInfo.Tag;
	
	
	if (g_GINavInfo.ResetFlagBack == 0)
	{
	  MEMCPY(&pNavResult->UtcTime, &g_GINavInfo.UtcTime, sizeof(UTC_T));
	  MEMCPY(&pNavResult->GpsTime, &g_GINavInfo.GPSTime, sizeof(GPST_T));
	  MEMCPY(&pNavResult->Position, &g_GINavInfo.Position, sizeof(POS_T));
	  MEMCPY(&pNavResult->Velocity, &g_GINavInfo.Velocity, sizeof(POS_T));
	
	  /*pNavResult->Attitude.Heading = g_GINavInfo.Euler.Phi;
	  pNavResult->Attitude.Pitch = g_GINavInfo.Euler.Theta;
	  pNavResult->Attitude.Roll = g_GINavInfo.Euler.Gamma;*/
		
		pNavResult->Attitude.Heading = g_GINavInfo.Euler.Phi;
	  pNavResult->Attitude.Pitch   = g_GINavInfo.Att_Pitch_Valid*DEG2RAD;
	  pNavResult->Attitude.Roll    = g_GINavInfo.Att_Roll_Valid*DEG2RAD;
	  pNavResult->UBI_Kind_Off     = g_GINavInfo.UBI_Kind_Off;
		
  	pNavResult->Acc_B_Num	  =   g_GINavInfo.Acc_D_Num;	
		
		pNavResult->GstDeta     =   g_GINavInfo.GstDeta;
		
  	pNavResult->Acc_C_Angle =   g_GINavInfo.Acc_C_Angle;
	  pNavResult->Acc_D_Angle =   g_GINavInfo.Acc_D_Angle;
	  pNavResult->Ini0_Kind   =   g_GINavInfo.Ini0_Kind;
	  pNavResult->Ini_Flag    =   g_GINavInfo.Ini_Flag;
	
	for (i = 0; i < 3; i++)
	{
		pNavResult->Gyro[i] = 0.0;
		pNavResult->Acc[i] = 0.0;
		for (j = 0; j < INS_UPDATE_SAMPLE_NUM; j++)
		{
			pNavResult->Gyro[i] += pImuData->Gyro[j][i];
			pNavResult->Acc[i] += pImuData->Acc[j][i];
		}
		pNavResult->Gyro[i] /= INS_UPDATE_SAMPLE_NUM*pImuData->MsrInterval / 1000.0;
		pNavResult->Acc[i] /= INS_UPDATE_SAMPLE_NUM*pImuData->MsrInterval / 1000.0;
	 }
	
	 pNavResult->PositionQuality = g_GINavInfo.PositionQuality;
	 pNavResult->INSState        = g_GINavInfo.INSState;	 
  } 

	pNavResult->M_City_Flag      =  g_GINavInfo.M_City_Flag;
	g_GINavInfo.ResetFlagBack    = 0;
	pNavResult->StaticFlag       = g_GINavInfo.StaticFlag;
	pNavResult->Install_Ini_Flag = g_GINavInfo.Install_Ini_Flag;
	pNavResult->OliData          =   g_GINavInfo.OliData;
	
 //----------------------------------------------------
 //	                        ���
 //----------------------------------------------------

	 memset(&IMUDataBuffer, 0, sizeof(IMU_FRAME_T));  //ȫ������
	
	
 //----------------------------------------------------
 //	               ���⵼��ģ��5HZ
 //----------------------------------------------------	

	if(pGnssData)
	{
     memset(&GNSSDataBuffer, 0, sizeof(GNSS_DATA_T));  //ȫ������
		
	   if((ZeroSecond==1)&&(g_GINavInfo.SYSFlag>=3))               //1HZ
		 {	
		   g_GINavInfo.GoodCount++;
		
			 if(g_GINavInfo.GoodCount>=9999)
			 {
			  g_GINavInfo.GoodCount=9999;
			 }
			 
			 pNavResult->GoodCount=g_GINavInfo.GoodCount;
		 }
		
		if((Debug_Flag)||(High_DGet_Flag))
		{
	   Gnss_Get_Flag  = 1;           //GNSS��ñ�־����������GNSS.... 5HZ  
		}
    else
		{			
     if(ZeroSecond)               //1HZ
		 {
       Gnss_Get_Flag  = 1;        //GNSS��ñ�־����������GNSS....     
     }
	  }     
	}

	//----------------------------------------------------
  //	               ����IMU���ݴ��
  //----------------------------------------------------	
	
	 GNSS_Run_Flag = 0;           //����GNSS���.......
		
	 GpsInsGetFlag = 1;           //GNSS��ɱ�־����������ǵ�һ֡��IMU���ݣ��Ӷ�����INS�Ƶ�.....

	return TRUE;
	
}

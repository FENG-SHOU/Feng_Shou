/******************************/
 /*Ó¦ÓÃNMEA0183±ê×¼Óï¾ä¸ñÊ½*/
/******************************/
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "GetGPS.h"
#include "hw_config.h"

#include "Delay.h"
#include "GetNass.h"

#include "SendGPS.h"
#include "GetIMU.h"




void HandleGpsData(uint8_t GpsData)
{
		 HandleGpsIni();		
		 
     HandleGGAOn(GpsData);
	   
		 SaveGpsData(GpsData);		
}



//-------------------------------
//void HandleGpsIni(void)
//-------------------------------
void HandleGpsIni(void)
{
	//------------------------------
  if(Gps_No_Num>=5)
  {	
   m_GSPData_Num  = 0;   //³õÊ¼»¯GPSÐÅÏ¢½ÓÊÕ                    --Sn	
	 GGA_Rx_Counter = 0;   //·Ç³£¹Ø¼ü........ÕâÑù¿ÉÒÔ±ÜÃâGGA_Rx_Counter³õÊ¼»¯²»¶Ô....
	 m_GSPData_First= 1;	 //µÚÒ»´Î»ñµÃÐ­Òé
  }
//------------------------------
 
   Gps_No_Num     = 0;   //Ö»ÒªÓÐÊý¾Ý½øÀ´£¬¾ÍÇåÁã..... 
}

//------------------------------------
//void SaveGpsData(uint8_t btData)
//------------------------------------
void SaveGpsData(uint8_t btData)
{	   
 		if(m_GSPData_Num>=NP_ALL_DATA_LEN)   //·ÀÖ¹Ô½½ç....ÒòÎªÄã²»ÖªµÀ¶Ô·½»áÒ»´ÎÐÔÓÐ¶àÉÙÊý¾Ý½øÀ´...
		{
     m_GSPData_Num=NP_ALL_DATA_LEN-1;
    }  	
		
	  m_GSPData[m_GSPData_Num]  =	btData;
		m_GSPData_Num++;
		
		if(m_GSPData_Num>=NP_ALL_DATA_LEN)   //·ÀÖ¹Ô½½ç....ÒòÎªÄã²»ÖªµÀ¶Ô·½»áÒ»´ÎÐÔÓÐ¶àÉÙÊý¾Ý½øÀ´...
		{
     m_GSPData_Num=NP_ALL_DATA_LEN-1;
    }  	
	
} 




//------------------------------------------
//void HandleGGAOn(unsigned char Comamnd)
//------------------------------------------
void HandleGGAOn(unsigned char Comamnd)
{
 static uint8_t i, utc_num=0;

 static uint8_t  GGA_TI_Flag=0;
	
	if(GGA_Rx_Counter>=sizeof(GGA_Rx_Data))       //±£»¤....
	{
   GGA_Rx_Counter=sizeof(GGA_Rx_Data)-1;
  }
	
  GGA_Rx_Data[GGA_Rx_Counter]=Comamnd;
  GGA_Rx_Counter++;
	
	
  if(Comamnd=='$')
	{
	  GGA_Rx_Data[0] = '$';
	  GGA_Rx_Counter = 1;
	}
	
  if(GGA_Rx_Counter==1)
  {
   if(GGA_Rx_Data[0]=='$')   //$GPGGA
   {
    GGA_Rx_Max   =  16;
    utc_num       =  0;		 
   }  
   else
   {
    GGA_Rx_Max    =  1;    
		GGA_Rx_Counter=  0;      //·Ç³£¹Ø¼ü........ÕâÑù¿ÉÒÔ100%»ñµÃÖ¡Í·...$..·ñÔò....
		utc_num       =  0;
   } 
  }
  else
  {
   if(GGA_Rx_Counter>=GGA_Rx_Max)
   {
     GGA_Rx_Counter = 0;      
   
		if(m_GSPData_First==1)
		{
		  if((GGA_Rx_Data[0]=='$')&&(GGA_Rx_Data[3]=='G')&&(GGA_Rx_Data[4]=='G')&&(GGA_Rx_Data[5]=='A'))
	    {
	     GGA_Ini_Flag    = 1;
		   GGA_TI_Flag     = 1;	 
			 m_GSPData_First = 0;
	    }
			
			if((GGA_Rx_Data[0]=='$')&&(GGA_Rx_Data[3]=='R')&&(GGA_Rx_Data[4]=='M')&&(GGA_Rx_Data[5]=='C'))
	    {
	     GGA_Ini_Flag    = 1;
		   GGA_TI_Flag     = 1;	 
			 m_GSPData_First = 0;
	    }
			
			if((GGA_Rx_Data[0]=='$')&&(GGA_Rx_Data[3]=='G')&&(GGA_Rx_Data[4]=='S')&&(GGA_Rx_Data[5]=='T'))
	    {
	     GGA_Ini_Flag    = 1;
		   GGA_TI_Flag     = 1;	 
			 m_GSPData_First = 0;
	    }		
		}			
    
		
	  //-------------------------------
		//           UTCÊ±¼ä
		//-------------------------------
		if(GGA_TI_Flag)
	  {
     GGA_TI_Flag=0;

    	for(i=0;i<6;i++)
			{
				if((GGA_Rx_Data[i+7]>=0x30)&&(GGA_Rx_Data[i+7]<=0x39))
				{
					utc_num++;
				}
			}
			
			for(i=0;i<2;i++)
			{
				if((GGA_Rx_Data[i+14]>=0x30)&&(GGA_Rx_Data[i+14]<=0x39))
				{
					utc_num++;
				}
			}
			
			if(utc_num==8)
			{
       IMDHour   = (GGA_Rx_Data[7]-0x30)*10+ (GGA_Rx_Data[8]-0x30);					//	Ê±¼ä¸³Óè
	     IMDMinute = (GGA_Rx_Data[9]-0x30)*10+ (GGA_Rx_Data[10]-0x30);				//	
	     IMDSecond = (GGA_Rx_Data[11]-0x30)*10+(GGA_Rx_Data[12]-0x30);			  //
       IMDMSecond= (GGA_Rx_Data[14]-0x30)*10+(GGA_Rx_Data[15]-0x30);			  //
				
			 if(IMDMSecond==0)	         //Èç¹ûºÁÃëÎªÁã£¬ÔòÇåÁãIMDMStime...
			 {
			  IMDMStime =0;
				GGA_Zero_Flag=1; 
			 }
       else
			 {
        if((IMDMSecond==20)||(IMDMSecond==40)||(IMDMSecond==60)||(IMDMSecond==80))
				{
          DetaTime =1000/5/BaseTime;      //Èç¹û5HZ....ãÐÖµ 
          Gps_Fren_Kind = 5;					    //Èç¹*û5HZ....Ôò¸Ð¾õGPSÇý¶¯Êä³ö...		
          IMDMStime			= IMDMSecond*10/BaseTime;
        }			
				
       }
			 
   		 
			 //------------------------------------
      }
    }	
		//-------------------------------
   }  
  }

	
	
}
//----------------------------------
//void  Gps_Data_Handle(void)
//----------------------------------
void  Gps_Receive_Handle(void)
{
	static uint16_t i;
	
	 Gps_No_Num++;
	
	if(Gps_No_Num>=5)
  {
		Gps_No_Num=5;     
			
		if((m_GSPData_Num>30))       //Èç¹û½ÓÊÕµ½µÄ×Ö·ûÊýÄ¿´ïµ½ãÐÖµ£¬ÔòÈÏÎª½ÓÊÕµ½ÕýÈ·µÄGPSÊý¾Ý
		{				
			 m_GSPData_NumB   = m_GSPData_Num;				
			 m_GSPData_Num=0;           //´Ë´¦ÇåÁã£¬ÒâÎ¶×ÅÖ»ÊÇ½øÐÐÒ»´ÎGps½ÓÊÕÊý¾ÝÅÐ¶Ï
				
        Gps_No_Data_Num=0;
 		  	//------------------------------------------------------ 
	      //               Ð­Òé½âÎö
				//------------------------------------------------------ 
				for(i=0;i<m_GSPData_NumB;i++)
				{
					ProcessNMEA(m_GSPData[i]);		
				}

				//------------------------------------------------------

				 GpsDriverAnalyseFlag = 1;     //GGAÐ­ÒéÇý¶¯GPSÐ­Òé½âÎö....Analyse_GpsData()..¸Ãº¯Êý½âÎöºÍGGAÒ»ÆðµÄÐ­Òé½âÎö...ºÍGGA²»Ò»ÆðµÄÐ­Òé£¬Ôò½âÎöÉÏÒ»
		

				if(Debug_Flag)
				{
				 Get_GGA_RMC_GST();
				}
				else
				{
				 if(RMC_Back_Flag)
				 {
				  Get_RMC_Data();
				 }
				
				 if(GGA_Zero_Flag)
				 {
					GGA_Zero_Flag = 0;
					Get_GSA_GAV();
				 }	
				}
				
		
			 //--------------------------------							
		 }
		 else
		 {
        m_GSPData_Num=0;           //´Ë´¦Ò²ÒªÇå³ý£¬·ÀÖ¹Ô½½ç....
     }
  }


	//-------------------------------------

}



///////////////////////////////////////////////////////////////////////////////
// ProcessNMEA: This method is the main state machine which processes individual
//				bytes from the buffer and parses a NMEA sentence. A typical
//				sentence is constructed as:
//
//					$CMD,DDDD,DDDD,....DD*CS<CR><LF>
//
//				Where:
//						'$'			HEX 24 Start of sentence
//						'CMD'		Address/NMEA command
//						',DDDD'		Zero or more data fields
//						'*CS'		Checksum field
//						<CR><LF>	Hex 0d 0A End of sentence
//	NMEA0183´¦ÀíÃüÁî,ÓÃÀ´´¦Àí´®¿Ú¼Ä´æÆ÷·¢À´µÄÊý¾Ýrx_data
//	Í¨¹ýswitchÃüÁîÀ´½â¾öÐ­ÒéÃ¿¸ö´úÂë¶ÎµÄÊý¾Ý´¦Àí					
///////////////////////////////////////////////////////////////////////////////
void ProcessNMEA(uint8_t btData)
{

static	NP_STATE    m_nState=NP_STATE_SOM;				    // Current state protocol parser is in ?????????????(??,??,??,???1,???2)
static  uint8_t     m_btChecksum=0;			    // Calculated NMEA sentence checksum
static  uint8_t     m_btReceivedChecksum=0;	    // Received NMEA sentence checksum (if exists)
static  uint16_t    m_wIndex=0;			      	// Index used for command and data

static  uint8_t     m_pCommand[NP_MAX_CMD_LEN];	    // NMEA command	 NMEA??
	
	switch(m_nState)
	{
		///////////////////////////////////////////////////////////////////////
		// Search for start of message '$'
		case NP_STATE_SOM :
			if(btData == '$')
			{
				m_btChecksum = 0;			     // ¸´Î»Ð£ÑéºÍ
				m_wIndex = 0;				       // ¸´Î»Ë÷Òý
				m_nState = NP_STATE_CMD;	 // Ö¸ÏòÏÂÒ»¸öcase
			}			    
		break;

		///////////////////////////////////////////////////////////////////////
		// Retrieve command (NMEA Address)
		case NP_STATE_CMD :
			if(btData != ',' && btData != '*')		  //Èç¹û²»ÊÇ¿ÕÊý¾Ý
			{
				m_pCommand[m_wIndex++] = btData;	 //½«Êý¾Ý´æÓÚÃüÁîÊý×éÖÐ

				m_btChecksum ^= btData;				 //°´Î»Òì»ò£¬¼´Ïà¼ÓÔËËã

				// Check for command overflow
				if(m_wIndex >= NP_MAX_CMD_LEN)		 //NP_MAX_CMD_LENÃüÁîÖ¡³¤¶È×î´ó8£¬Èç¹û´óÓÚËµÃ÷Êý¾ÝÎÞÐ§
				{
					m_nState = NP_STATE_SOM;		 //ÖØÐÂ»Øµ½µÚÒ»case
				}
			}
			else								     //Èç¹ûÊÇ','»òÕß'*'£¬ÄÇÃ´»»ÏÂÒ»¸öÊý¾ÝÓï¾ä
			{
				m_pCommand[m_wIndex] = '\0';	     //terminate command Ìí¼Ó×Ö·û´®½áÊø·û
				m_btChecksum ^= btData;
				m_wIndex = 0;
     

				m_nState = NP_STATE_DATA;		// goto get data state
			}
		break;

		///////////////////////////////////////////////////////////////////////
		// Store data and check for end of sentence or checksum flag
		case NP_STATE_DATA :
			
		    if(m_wIndex >= NP_MAX_DATA_LEN-1) // Check for buffer overflow
				{
				   m_nState = NP_STATE_SOM;
				}
				
			if(btData == '*') // checksum flag?	 Èç¹ûÃ»ÓÐÊý¾Ý´úÂë¶Î£¬ÄÇÃ´»áÖ±½Ó³öÏÖÏÂ¸öÐ£ÑéºÍ´úÂë¶Î 
			{
	     
				if(m_wIndex>=NP_MAX_DATA_LEN-1)     //±£»¤.....
				{
         m_wIndex=NP_MAX_DATA_LEN-1;
        }
				
				m_GPSpData[m_wIndex] = '\0';		
  
				  
				m_nState = NP_STATE_CHECKSUM_1;	  //Ð£ÑéºÍ1
				
			}
			else // no checksum flag, store data
			{
				//
				// Check for end of sentence with no checksum
				//
				
				if(m_wIndex>=NP_MAX_DATA_LEN-1)     //±£»¤.....
				{
         m_wIndex=NP_MAX_DATA_LEN-1;
        }
				
				if(btData == '\r')	              //»Ø³µ·û
				{

				  m_GPSpData[m_wIndex] = '\0';				
				 		
				 	m_nState = NP_STATE_SOM;
					return;			 //£¿£¿£¿
				}
				//
				// Store data and calculate checksum
				//
				m_btChecksum ^= btData;

	
				m_GPSpData[m_wIndex] = btData;							
				

				if(++m_wIndex >= NP_MAX_DATA_LEN) // Check for buffer overflow
				{
				 m_nState = NP_STATE_SOM;
				}
			}
		break;

		///////////////////////////////////////////////////////////////////////
		case NP_STATE_CHECKSUM_1 :	

		 if( (btData - '0') <= 9)
			{
				m_btReceivedChecksum = (btData - '0') << 4;
			}
			else
			{
				m_btReceivedChecksum = (btData - 'A' + 10) << 4;
			}

			m_nState = NP_STATE_CHECKSUM_2;																   

		break;

		///////////////////////////////////////////////////////////////////////
		case NP_STATE_CHECKSUM_2 :
			
			if( (btData - '0') <= 9)
			{
				m_btReceivedChecksum |= (btData - '0');
			}
			else
			{
				m_btReceivedChecksum |= (btData - 'A' + 10);
			}

			if(m_btChecksum == m_btReceivedChecksum)
			{
			 	 ProcessCommand(m_pCommand);	
			}
				 
			m_nState = NP_STATE_SOM;                      //ÖØÐÂ¿ªÊ¼Ð£ÑéÏÂÒ»ÌõÐ­Òé

			break;

		///////////////////////////////////////////////////////////////////////
		default : m_nState = NP_STATE_SOM;
	}
 }
 
uint8_t ProcessCommand(uint8_t *pCommand)
{
	//
	// GPGGA
	//
	if( strcmp((char *)pCommand, "GPGGA") == NULL )		//Èç¹û×Ö·û´®·ûºÏ£¬Ôò·µ»ØNULL=0
	{
	 ProcessGPGGA(m_GPSpData);  					
   GPGGAData.Gp_PN_Kind=1; 
	}
	else if( strcmp((char *)pCommand, "GLGGA") == NULL )		//Èç¹û×Ö·û´®·ûºÏ£¬Ôò·µ»ØNULL=0
	{
	 ProcessGPGGA(m_GPSpData); 
   GPGGAData.Gp_PN_Kind=2;	
	}	
	else if( strcmp((char *)pCommand, "BDGGA") == NULL )		//Èç¹û×Ö·û´®·ûºÏ£¬Ôò·µ»ØNULL=0
	{
	 ProcessGPGGA(m_GPSpData); 
   GPGGAData.Gp_PN_Kind=3;	
	}	
	else if( strcmp((char *)pCommand, "GNGGA") == NULL )		//Èç¹û×Ö·û´®·ûºÏ£¬Ôò·µ»ØNULL=0
	{
	 ProcessGPGGA(m_GPSpData); 
   GPGGAData.Gp_PN_Kind=4;	
	}	
 //-------------------------------------------------------	
	else if( strcmp((char *)pCommand, "GPRMC") == NULL )
	{
	 ProcessGPRMC(m_GPSpData); 		
	  GPRMCData.Gp_PN_Kind=1;	
	}
	else if( strcmp((char *)pCommand, "GLRMC") == NULL )
	{
	 ProcessGPRMC(m_GPSpData);	
	  GPRMCData.Gp_PN_Kind=2;				
	}
	else if( strcmp((char *)pCommand, "BDRMC") == NULL )
	{
	 ProcessGPRMC(m_GPSpData);	
	  GPRMCData.Gp_PN_Kind=3;				
	}
	else if( strcmp((char *)pCommand, "GNRMC") == NULL )
	{
	 ProcessGPRMC(m_GPSpData);	
	  GPRMCData.Gp_PN_Kind=4;				
	}
 //-------------------------------------------------------
	else if( strcmp((char *)pCommand, "GPVTG") == NULL )
	{
	 ProcessGPVTG(m_GPSpData); 		
	  GPVTGData.Gp_PN_Kind=1;	
	}
	else if( strcmp((char *)pCommand, "GLVTG") == NULL )
	{
	 ProcessGPVTG(m_GPSpData); 	
	  GPVTGData.Gp_PN_Kind=2;			
	}
	else if( strcmp((char *)pCommand, "BDVTG") == NULL )
	{
	 ProcessGPVTG(m_GPSpData); 	
	  GPVTGData.Gp_PN_Kind=3;			
	}
	else if( strcmp((char *)pCommand, "GNVTG") == NULL )
	{
	 ProcessGPVTG(m_GPSpData); 	
	  GPVTGData.Gp_PN_Kind=4;			
	}

//------------------------------------------------------
 	//------------------------------------------------------
	else if( strcmp((char *)pCommand, "GPZDA") == NULL )
	{
	 ProcessGPZDA(m_GPSpData);		
	 GPZDAData.GpZDA_PN_Kind=1;	
	}
	else if( strcmp((char *)pCommand, "GLZDA") == NULL )
	{
	 ProcessGPZDA(m_GPSpData);	
   GPZDAData.GpZDA_PN_Kind=2;			
	}
	else if( strcmp((char *)pCommand, "BDZDA") == NULL )
	{
	 ProcessGPZDA(m_GPSpData);	
   GPZDAData.GpZDA_PN_Kind=3;			
	}
	else if( strcmp((char *)pCommand, "GNZDA") == NULL )
	{
	 ProcessGPZDA(m_GPSpData);	
   GPZDAData.GpZDA_PN_Kind=4;			
	}
	 //-------------------------------------------------------
  else if( strcmp((char *)pCommand, "GPGST") == NULL )
	{
	 ProcessGPGST(m_GPSpData);	
	}
	else if( strcmp((char *)pCommand, "GNGST") == NULL )
	{
	 ProcessGPGST(m_GPSpData);			
	}
	else if( strcmp((char *)pCommand, "GLGST") == NULL )
	{
	 ProcessGPGST(m_GPSpData);	
	}
	else if( strcmp((char *)pCommand, "BDGST") == NULL )
	{
	 ProcessGPGST(m_GPSpData);			
	}
	
	//-------------------------------------------------------
  else if( strcmp((char *)pCommand, "GPGSA") == NULL )
	{
	 ProcessGPGSA(m_GPSpData);	
	}
	else if( strcmp((char *)pCommand, "GNGSA") == NULL )
	{
	 ProcessGPGSA(m_GPSpData);			
	}
	else if( strcmp((char *)pCommand, "GLGSA") == NULL )
	{
	 ProcessGPGSA(m_GPSpData);	
	}
	else if( strcmp((char *)pCommand, "BDGSA") == NULL )
	{
	 ProcessGPGSA(m_GPSpData);			
	}
	//-------------------------------------------------------
	else if( strcmp((char *)pCommand, "GPGSV") == NULL )
	{
	 ProcessGPGSV(m_GPSpData);		
	}
	else if( strcmp((char *)pCommand, "GNGSV") == NULL )
	{
	 ProcessGPGSV(m_GPSpData);		
	}
	else if( strcmp((char *)pCommand, "GLGSV") == NULL )
	{
	 ProcessGPGSV(m_GPSpData);		
	}
	else if( strcmp((char *)pCommand, "BDGSV") == NULL )
	{
	 ProcessGPGSV(m_GPSpData);		
	}
	return TRUE;
}

//---------------------------------
//void Get_GGA_RMC_GST(void)
//---------------------------------
void Get_RMC_Data(void)
{

 uint16_t i=0,GStart_Num=0, RStart_Num=0;

	
 for(i=5;i<m_GSPData_NumB;i++)
 {	
//-------------------------------------------------------------------------------------------------------------------	 
  if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='G')&&(m_GSPData[i]=='A'))
  {
	  GStart_Num=i-5;	 
		m_GSPData[i-3]='I';
	}
	//-------------------------------------------------------------------------------------------------------------------
	 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='R')&&(m_GSPData[i-1]=='M')&&(m_GSPData[i]=='C'))
  {
	  RStart_Num=i-5;	 
		m_GSPData[i-3]='I';
	}	
 }
 
 

  //--------------------------------------------------	 
	if(GStart_Num>RStart_Num)     //Èç¹ûRMCÔÚÇ°£¬Ôò´ÓRMC¿ªÊ¼
	{
   GGARMC_Length = GStart_Num - RStart_Num;
  	 
   for(i=RStart_Num;i<GStart_Num;i++)
   {   
    m_RMCpData[i-RStart_Num]=m_GSPData[i];
   }  

  }

}



//---------------------------------
//void Get_GGA_RMC_GST(void)
//---------------------------------
void Get_GGA_RMC_GST(void)
{

 uint16_t i=0,GStart_Num=0, RStart_Num=0,ZStart_Num=0,End_Num=0,GST_Num=0;
 uint8_t Gsa_First=1;
 uint8_t  Gsv_Flag=0;
	
 GStart_Num = 0;
 RStart_Num = 0;
	
 End_Num   = 0;
 Gsa_First = 1;
	
 for(i=5;i<m_GSPData_NumB;i++)
 {	
//-------------------------------------------------------------------------------------------------------------------	 
  if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='G')&&(m_GSPData[i]=='A'))
  {
	  GStart_Num=i-5;	 
		m_GSPData[i-3]='I';
	}
	//-------------------------------------------------------------------------------------------------------------------
	 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='R')&&(m_GSPData[i-1]=='M')&&(m_GSPData[i]=='C'))
  {
	  RStart_Num=i-5;	 
		m_GSPData[i-3]='I';
	}

	//-------------------------------------------------------------------------------------------------------------------
 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='S')&&(m_GSPData[i]=='A'))
  {	 
		Gsv_Flag=1;
		
	 if(Gsa_First==1)
	 {		 
		Gsa_First=0;
	  End_Num=i-5;
	 }
	}
	
		//-------------------------------------------------------------------------------------------------------------------
 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='S')&&(m_GSPData[i]=='V'))
  {	 
		Gsv_Flag=1;
		
	 if(Gsa_First==1)
	 {		 
		Gsa_First=0;
	  End_Num=i-5;
	 }
	}
//-------------------------------------------------------------------------------------------------------------------
 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='S')&&(m_GSPData[i]=='T'))
  {	 
   GST_Num=i-5;
		
	 if(Gsv_Flag==0)   //Èç¹ûÃ»ÓÐGSV. GSA..
	 {
		End_Num=i-5;
	 }
	 
	 m_GSPData[i-3]='I';
	}	

	//-------------------------------------------------------------------------------------------------------------------
	if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='Z')&&(m_GSPData[i-1]=='D')&&(m_GSPData[i]=='A'))
  {
	  ZStart_Num=i-5;	 
		m_GSPData[i-3]='I';
	}
	
}
 
 
 if((GStart_Num!=RStart_Num)&&(End_Num>0)&&(GStart_Num<End_Num))
 {
  //--------------------------------------------------	 
	if(GStart_Num>RStart_Num)     //Èç¹ûRMCÔÚÇ°£¬Ôò´ÓRMC¿ªÊ¼
	{
   GGARMC_Length = End_Num - RStart_Num;
  	 
   for(i=RStart_Num;i<End_Num;i++)
   {   
    m_GSPDataB[i-RStart_Num]=m_GSPData[i];
   }  
 }
	else                          //Èç¹ûGGAÔÚÇ°£¬Ôò´ÓGGA¿ªÊ¼
	{
   GGARMC_Length = End_Num - GStart_Num;
  
	 for(i=GStart_Num;i<End_Num;i++)
   {   
    m_GSPDataB[i-GStart_Num]=m_GSPData[i];
   }  
  }

	//--------------------------------------------------	 	
	if((ZStart_Num==0)||(GST_Num>ZStart_Num))
	{	
	 for(i=GST_Num;i<m_GSPData_NumB;i++)
   {   
    m_GSPDataB[GGARMC_Length]=m_GSPData[i];
	  GGARMC_Length++;
   }
  }
	else
	{
	 for(i=GST_Num;i<ZStart_Num;i++)
   {   
    m_GSPDataB[GGARMC_Length]=m_GSPData[i];
	  GGARMC_Length++;
   }
	
	}
	
 	GGARMCGST_Length=GGARMC_Length;	
 }
 else
 {
   GGARMCGST_Length =0;
 }

 
 
}


void Get_GSA_GAV(void)
{

 uint16_t i,Start_Num=0,End_Num=0;
 uint8_t Gsa_First=1;
	
 Start_Num=0;
 End_Num =0;
 Gsa_First=1;
	
 for(i=5;i<m_GSPData_NumB;i++)
 {
	 
  if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='S')&&(m_GSPData[i]=='A'))
  {
	 if(Gsa_First==1)
	 {		 
		Gsa_First=0;
	  Start_Num=i-5;
	 }
	}

 if((m_GSPData[i-5]=='$')&&(m_GSPData[i-4]=='G')&&(m_GSPData[i-2]=='G')&&(m_GSPData[i-1]=='S')&&(m_GSPData[i]=='T'))
  {
	  End_Num=i-5;
	} 	
 }
 
 
 if((Start_Num>0)&&(End_Num>0)&&(Start_Num<End_Num))
 {  
	 GSA_Length = End_Num - Start_Num;
	 
  for(i=Start_Num;i<End_Num;i++)
  {   
   m_GSPDataB[i-Start_Num]=m_GSPData[i];
  }
	
 }
 else
 {
   GSA_Length =0;
 }
}

/*-------------------USB??------------------------------*/



void Send_GPSData(void)
{ 	
	if(GpsDriverSendFlag&&Debug_Flag)                          //??GPS......
	{
		GpsDriverSendFlag=0;
		
		DMA_Send_Kind =1;                     //???????1,GPS
		
		DMA_Tx_Num   = GGARMCGST_Length;
		
  
	  if(DMA_Tx_Num>0)
		{
		 USART_DMA_TX_Configuration(m_GSPDataB);
		
		 SendDriver();
		}

	}
}


//************************************************************
//                        GPS½âÂë
//************************************************************

 ///////////////////////////////////////////////////////////////////////////////
// Name:		GetField
//
// Description:	This function will get the specified field in a NMEA string.
//
// Entry:		uint8_t *pData -		Pointer to NMEA string	     Ö¸ÏòÊý¾Ý×Ö¶Î
//				uint8_t *pField -		pointer to returned field	 Ö¸Ïò»ØËÝ×Ö¶Î
//				int32_t nfieldNum -		Field offset to get
//				int32_t nMaxFieldLen -	Maximum of bytes pFiled can handle
//      Õâ¸öº¯ÊýÄÜ¹»»ñÈ¡Ö¸¶¨×Ö¶ÎµÄÊýÖµ£¬Èç¹û×Ö¶Î·ûºÏ£¬·µ»ØÖµÎª1£¬·ñÔòÎª0,¾ßÌåÊ¹ÓÃ·½·¨¿É¼ûÏÂÒ»¸öº¯ÊýProcessGPZDA
///////////////////////////////////////////////////////////////////////////////
uint8_t GetField(uint8_t *pData, uint8_t *pField, int32_t nFieldNum, int32_t nMaxFieldLen)
{
	
	int32_t    Get_i;
  int32_t    Get_i2;
  int32_t    Get_nField;

	//
	// Validate params
	//
	if(pData == NULL || pField == NULL || nMaxFieldLen <= 0)
	{
		return FALSE;		   //º¯ÊýÁ¢¼´½áÊø£¬·µ»Ø0
	}

	//
	// Go to the beginning of the selected field
	//
	Get_i = 0;
	Get_nField = 0;
	while(Get_nField != nFieldNum && pData[Get_i])	//Ö±µ½nfiledµ½´ïnFieldNum£¬while½áÊø,nFieldNum¾ÍÊÇ¸ø³öµÄÒªµ½´ïµÄ×Ö¶ÎµÄÐòºÅ
	{
		if(pData[Get_i] == ',')		            //Óöµ½¶ººÅËµÃ÷ÒÑ¾­Ìø¹ýÒ»¸ö×Ö¶Î£¬nField×Ô¼Ó1
		{
			Get_nField++;
		}

		Get_i++;
		
			
		if(Get_i >=NP_MAX_DATA_LEN)               //Èç¹ûGet_i³¬¹ýÊý×é³¤¶È£¬Ôò....ÍË³ö...20140729.....
		{
	    pField[0]  = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø			
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý		
    }

		if(pData[Get_i] == NULL)
		{
			pField[0] = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý
		}
	}

	if(pData[Get_i] == ',' || pData[Get_i] == '*')
	{
		pField[0] = '\0';			   //ÈôÖ¸¶¨×Ö¶ÎÒ²ÎÞÊýÖµ£¬Îª,»òÕß*ºÅÔò£¬
		return FALSE;				   //Èç¹ûÌø³öwhileºóµÄµÚÒ»¸ö×Ö·û»¹ÊÇ,»òÕß*£¬·µ»Ø´íÎó
	}

	//
	// copy field from pData to Field
	//
	Get_i2 = 0;
	while(pData[Get_i] != ',' && pData[Get_i] != '*' && pData[Get_i])  //ÈôÖ¸¶¨×Ö¶ÎÓÐÊýÖµ
	{
		pField[Get_i2] = pData[Get_i];				//½«Æä´æ·ÅÓÚpDataÖÐ
		Get_i2++; Get_i++;

		//
		// check if field is too big to fit on passed parameter. If it is,
		// crop returned field to its max length.
		//
		if(Get_i2 >= nMaxFieldLen)			 //Èç¹ûi2´óÓÚ¸Ã×Ö¶ÎÓ¦ÓÐµÄ×î´ó³¤¶È£¬ÄÇÃ´ÈÃi2µÈÓÚ¸Ã×î´ó³¤¶È
		{								 //ÒòÎªÔÚ´¦ÀíÄ³Ð©×Ö¶ÎÊ±£¬×Ö¶Î»á¶à·¢Êý¾Ý£¬ÎªÁË±£Ö¤Êý¾ÝµÄ×¼È·
			Get_i2 = nMaxFieldLen-1;		 //ÕâÀï¹æ¶¨ÁËi2µÄ³¤¶È£¬Ò²¾Í¹æ¶¨ÁË×îºó·µ»Ø×Ö¶ÎµÄ³¤¶È
			break;
		}
		
		if(Get_i >=NP_MAX_DATA_LEN)               //Èç¹ûGet_i³¬¹ýÊý×é³¤¶È£¬Ôò....ÍË³ö...20140729.....
		{
	    pField[0]  = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø			
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý		
    }
	}
	pField[Get_i2] = '\0';

	return TRUE;						 //Èç¹ûÕû¸öº¯ÊýÔËÐÐÍê³É£¬×Ö¶Î·µ»Ø³É¹¦£¬ÄÇÃ´·µ»Øtrue=1
}




//----------------------------------
//uint8_t DataJudge(uint8_t GetData)
//----------------------------------
uint8_t DataJudge(uint8_t *pFieldData)
{
	uint8_t D_i=0,PDataT=0,DFlag=1;
		
	for(D_i=0;D_i<MAXFIELD;D_i++)
	{
		PDataT=pFieldData[D_i];
		
	 if(PDataT!='\0')
	 {
    if(((PDataT>=0x30)&&(PDataT<=0x39))||(PDataT=='.'))
	  {
     DFlag=1;
    }
	  else
	  {
     DFlag=0;
		 break;
    }
   }
	 else
	 {
		 if(D_i==0)
		 {
      DFlag=0;
     }		 
    break;
   }
 } 
 return  DFlag;	
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void ProcessGPGGA(uint8_t *pData)
{
uint8_t     GGApField[MAXFIELD];
char        GGApBuff[MAXFIELD];
	
static  uint8_t Gps_GError_Flag=0;
	//
	// Time
	//
	if(GetField(pData, GGApField, 0, MAXFIELD))
	{
		// Hour
		GGApBuff[0] = GGApField[0];
		GGApBuff[1] = GGApField[1];
		GGApBuff[2] = '\0';
		GPGGAData.Hour = atoi(GGApBuff);		

		// minute
		GGApBuff[0] = GGApField[2];
		GGApBuff[1] = GGApField[3];
		GGApBuff[2] = '\0';
		GPGGAData.Minute = atoi(GGApBuff);

		// Second
		GGApBuff[0] = GGApField[4];
		GGApBuff[1] = GGApField[5];
		GGApBuff[2] = '\0';
		GPGGAData.Second = atoi(GGApBuff);

		// Second
		GGApBuff[0] = GGApField[7];
		GGApBuff[1] = GGApField[8];
		GGApBuff[2] = GGApField[9];
		GGApBuff[3] = '\0';
		GPGGAData.MSecond = atoi(GGApBuff);
		
		if((GPGGAData.Hour==0)&&(GPGGAData.Minute==0)&&(GPGGAData.Second==0))
		{
		 GPGGAData.Utc_Flag=0;
		}
		else
		{
		 GPGGAData.Utc_Flag=1;
		}
	}
	else
	{
	  GPGGAData.Utc_Flag=0;
	}

	
	//
	// Latitude
	//
	if(GetField(pData, GGApField, 1, MAXFIELD))
	{

		
		if(DataJudge(GGApField))
		{
		 GPGGAData.Latitude  = atof((char *)GGApField+2) / 60.0;
		 GGApField[2] = '\0';
		 GPGGAData.Latitude += atof((char *)GGApField);		
		}
    else
	  {
     GPGGAData.Latitude=0.0;
    }
	}
  else
	{
    GPGGAData.Latitude=0.0;
  }

	
	if(GetField(pData, GGApField, 2, MAXFIELD))
	{
	
	   GPGGAData.SNth=GGApField[0];

	   if(GGApField[0] == 'S')
	   {
		  GPGGAData.Latitude = -GPGGAData.Latitude;
	   }
	}
	


	//------------------------------------------------
	// Longitude
	//------------------------------------------------
	if(GetField(pData, GGApField, 3, MAXFIELD))
	{
		if(DataJudge(GGApField))
		{
		 GPGGAData.Longitude = atof((char *)GGApField+3) / 60.0;
		 GGApField[3] = '\0';
		 GPGGAData.Longitude += atof((char *)GGApField);	
		}
		else
		{
     GPGGAData.Longitude=0.0;
    }
	}
	else
	{
    GPGGAData.Longitude=0.0;
  }

	
	if(GetField(pData, GGApField, 4, MAXFIELD))
	{
	  GPGGAData.WEst= GGApField[0]; 

		if(GGApField[0] == 'W')
		{
			GPGGAData.Longitude = -GPGGAData.Longitude;
		}	 
	}

		//------------------------------------------------
	// GPS quality
	//------------------------------------------------
	if(GetField(pData, GGApField, 5, MAXFIELD))
	{
		GPGGAData.GPSQuality = GGApField[0] - '0';
	}
	else
	{
	  GPGGAData.GPSQuality  = 0;
	}


	//------------------------------------------------
	// Satellites in use
	//------------------------------------------------
	if(GetField(pData, GGApField, 6, MAXFIELD))
	{
		GGApBuff[0] = GGApField[0];
		GGApBuff[1] = GGApField[1];
		GGApBuff[2] = '\0';
		GPGGAData.NumOfSatsInUse = atoi(GGApBuff);
	}
	else
	{
	  GPGGAData.NumOfSatsInUse =0;
	}

	//------------------------------------------------
	// HDOP
    //------------------------------------------------
	if(GetField(pData, GGApField, 7, MAXFIELD))
	{
		GPGGAData.HDOP = atof((char *)GGApField);
	}


	//------------------------------------------------
	// Altitude
	//------------------------------------------------
	if(GetField(pData, GGApField, 8, MAXFIELD))
	{
		if(DataJudge(GGApField))
		{
		 GPGGAData.Altitude = atof((char *)GGApField);
		}
	}

	//------------------------------------------------
	// M
	//------------------------------------------------
	if(GetField(pData, GGApField, 9, MAXFIELD))
	{
	    GPGGAData.Auint= GGApField[0]; 
	}

	//------------------------------------------------
	//Geoidal
	//------------------------------------------------
	if(GetField(pData, GGApField, 10, MAXFIELD))
	{
	  GPGGAData.Geoidal = atof((char *)GGApField);
	}

	//------------------------------------------------
	// M
	//------------------------------------------------
	if(GetField(pData, GGApField, 11, MAXFIELD))
	{
	  GPGGAData.Guint= GGApField[0]; 
	}
	
	//------------------------------------------------
	//Age of Diff. Corr
	//------------------------------------------------
	if(GetField(pData, GGApField, 12, MAXFIELD))
	{
	  GPGGAData.DTime = atof((char *)GGApField);
		GetAgeDataFlag  = 1;   //
	}
	
	 
	//------------------------------------------------
	//Dif. Ref. Station ID
	//------------------------------------------------
	if(GetField(pData, GGApField, 13, MAXFIELD))
	{
	  GPGGAData.DGpsID = atoi((char *)GGApField);	
		GPGGAData.IDValid=1;
		
		GPGGAData.DFID_Data[0] = GGApField[0];
		GPGGAData.DFID_Data[1] = GGApField[1];
		GPGGAData.DFID_Data[2] = GGApField[2];
		GPGGAData.DFID_Data[3] = GGApField[3];
	}
	else
	{
    GPGGAData.IDValid=0;
		GPGGAData.DFID_Data[0] ='0';
		GPGGAData.DFID_Data[1] ='0';
		GPGGAData.DFID_Data[2] ='0';
		GPGGAData.DFID_Data[3] ='0';	
		
  }	  

   if((GPGGAData.SNth!='S')&&(GPGGAData.SNth!='N'))
   {   
   	Gps_GError_Flag=1;
   }

   if((GPGGAData.WEst!='W')&&(GPGGAData.WEst!='E'))
   {					   
   	Gps_GError_Flag=1;  
   }

   if((GPGGAData.Auint!='M')||(GPGGAData.Guint!='M'))
   {					   
 	  Gps_GError_Flag=1;  
   }
	 

//  if(Gps_GError_Flag==0)
   {
 	  GPGGAData.Count++;
	  GGA_Get_Flag  = 1;                 //»ñµÃÊý¾Ý---´«ÊäÊý¾Ý  
   }
  
   Gps_GError_Flag = 0;                 //ÇåÁã
}



//------------------------------------
//void ProcessGPRMC(uint8_t *pData)
//------------------------------------
void ProcessGPRMC(uint8_t *pData)
{
uint8_t     RMCpField[MAXFIELD];
char        RMCpBuff[MAXFIELD];
static  uint8_t Gps_GError_Flag=0;
	//
	// Time
	//
	if(GetField(pData, RMCpField, 0, MAXFIELD))
	{
		// Hour
		RMCpBuff[0] = RMCpField[0];
		RMCpBuff[1] = RMCpField[1];
		RMCpBuff[2] = '\0';
		GPRMCData.Hour = atoi(RMCpBuff);

		// minute
		RMCpBuff[0] = RMCpField[2];
		RMCpBuff[1] = RMCpField[3];
		RMCpBuff[2] = '\0';
		GPRMCData.Minute = atoi(RMCpBuff);

		// Second
		RMCpBuff[0] = RMCpField[4];
		RMCpBuff[1] = RMCpField[5];
		RMCpBuff[2] = '\0';
		GPRMCData.Second = atoi(RMCpBuff);
		
	 // MSecond
		RMCpBuff[0] = RMCpField[7];
		RMCpBuff[1] = RMCpField[8];
		RMCpBuff[2] = RMCpField[9];
		RMCpBuff[3] = '\0';
		GPRMCData.MSecond = atoi(RMCpBuff);
	}

	//
	// Data valid
	//
	if(GetField(pData, RMCpField, 1, MAXFIELD))
	{
		GPRMCData.DataValid = RMCpField[0];
	}
	
	//
	// latitude
	//
	if(GetField(pData, RMCpField, 2, MAXFIELD))
	{

		if(DataJudge(RMCpField))
		{
		 GPRMCData.Latitude = atof((char *)RMCpField+2) / 60.0;    //°Ñ×Ö·û´®×ª»»³É¸¡µãÊý
		 RMCpField[2] = '\0';
		 GPRMCData.Latitude += atof((char *)RMCpField);
		}
		else
		{
			GPRMCData.Latitude=0.0;   //GetNass´¦Àí....
		}
	}
	else
	{
		GPRMCData.Latitude=0.0;   //GetNass´¦Àí....
	}
	
	if(GetField(pData, RMCpField, 3, MAXFIELD))
	{
				
	   GPRMCData.SNth=RMCpField[0];

		if(GPRMCData.SNth == 'S')
		{
		  GPRMCData.Latitude = -GPRMCData.Latitude;
		}
	 
	}

	//
	// Longitude
	//
	if(GetField(pData, RMCpField, 4, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 GPRMCData.Longitude = atof((char *)RMCpField+3) / 60.0;
		 RMCpField[3] = '\0';
		 GPRMCData.Longitude += atof((char *)RMCpField);
		}
		else
		{
		 GPRMCData.Longitude=0.0;  //GetNass´¦Àí....
		}
	}
	else
	{
	 GPRMCData.Longitude=0.0;  //GetNass´¦Àí....
	}
		

	if(GetField(pData, RMCpField, 5, MAXFIELD))
	{

	   GPRMCData.WEst=RMCpField[0];

		if(GPRMCData.WEst == 'W')
		{
			GPRMCData.Longitude = -GPRMCData.Longitude;
		}
	 
	}
	   
	//
	// Date
	//
	if(GetField(pData, RMCpField, 8, MAXFIELD))
	{
		// Day
		RMCpBuff[0] = RMCpField[0];
		RMCpBuff[1] = RMCpField[1];
		RMCpBuff[2] = '\0';
		GPRMCData.Day = atoi(RMCpBuff);

		// Month
		RMCpBuff[0] = RMCpField[2];
		RMCpBuff[1] = RMCpField[3];
		RMCpBuff[2] = '\0';
		GPRMCData.Month = atoi(RMCpBuff);

		// Year (Only two digits. I wonder why?)
		RMCpBuff[0] = RMCpField[4];
		RMCpBuff[1] = RMCpField[5];
		RMCpBuff[2] = '\0';
		GPRMCData.Year = atoi(RMCpBuff);
		GPRMCData.Year += 2000;				// make 4 digit date -- What assumptions should be made here?
	}

	//
	// course over ground, degrees true
	//
	if(GetField(pData, RMCpField, 9, MAXFIELD))
	{
  		
		GPRMCData.MagVar = atof((char *)RMCpField);
		GPRMCData.MagVarValid=1;
	
	}
	else
	{
	  GPRMCData.MagVarValid=0;
  }
	
	if(GetField(pData, RMCpField, 10, MAXFIELD))
	{
	  if(RMCpField[0] == 'W')
	   {
	    GPRMCData.MagVar = -GPRMCData.MagVar;
	   }
		 
		 GPRMCData.MagWEst=RMCpField[0];
	}

	if(GetField(pData, RMCpField, 11, MAXFIELD))
	{
	   GPRMCData.ModeIn = RMCpField[0];	   
	}
	
	//
	// Ground speed
	//
	if(GetField(pData, RMCpField, 6, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 GPRMCData.GroundSpeed = atof((char *)RMCpField);
		 GPRMCData.SpeedValid = 1;
		}
		else
		{
		 GPRMCData.CourseValid=0;
     GPRMCData.ModeIn='N';
    }
	}
	else
	{
		GPRMCData.SpeedValid  = 0;
	}

	//
	// course over ground, degrees true
	//
	if(GetField(pData, RMCpField, 7, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 GPRMCData.Course = atof((char *)RMCpField);
		 GPRMCData.CourseValid=1;
		}
		else
		{
		 GPRMCData.CourseValid=0;
     GPRMCData.ModeIn='N';
    }
	}
	else
	{
	  GPRMCData.CourseValid=0;
  }
  //--------------------------------------------------
  // ¾À´í
  //--------------------------------------------------

  if((GPRMCData.SNth!='S')&&(GPRMCData.SNth!='N'))
   {   
   	Gps_GError_Flag=1;
   }

   if((GPRMCData.WEst!='W')&&(GPRMCData.WEst!='E'))
   {					   
 	  Gps_GError_Flag=1;  
   }
	 
	// if(Gps_GError_Flag==0)
   {
  	GPRMCData.Count++;   							
	  RMC_Get_Flag  = 1;						  //	
	  Rmc_Vtg_Flag  = 1;              // RMCÄ£Ê½...  
   }

		 
    Gps_GError_Flag  = 0;
	 
	
}



//-------------------------------------
//void ProcessGPVTG(uint8_t *pData)
//-------------------------------------
void ProcessGPVTG(uint8_t *pData)
{
	 uint8_t     VTGpField[MAXFIELD];

   uint8_t     Gps_GError_Flag=0;
	
	//
	// ÔË¶¯½Ç¶È 
	//
	if(GetField(pData, VTGpField, 0, MAXFIELD))	  //·½Ïò  
	{
		 if(DataJudge(VTGpField))
		 {
	    GPVTGData.Ttrack = atof((char *)VTGpField);
			GPVTGData.CourseValid=1; 
		 }
		 else
		 {
      GPVTGData.Ttrack =0.0;
			GPVTGData.CourseValid=0; 
     }
	}
	else
	{
	   GPVTGData.Ttrack =0;	
     GPVTGData.CourseValid = 0; 		
	}

	//Õæ±±²ÎÕÕÏµ
    if(GetField(pData, VTGpField, 1, MAXFIELD))	  //·½Ïò  T
	{
	   GPVTGData.Tax = VTGpField[0];	
	}

	//ÔË¶¯½Ç¶È 
	if(GetField(pData, VTGpField, 2, MAXFIELD))	  //ËÙ¶È
	{
		if(DataJudge(VTGpField))
		{
	   GPVTGData.Mtrack = atof((char *)VTGpField);
		} 
   else
   {
     GPVTGData.Mtrack = 0; 
   }	
	}
	else
	{
	   GPVTGData.Mtrack =0;		
	}

    //´Å±±²ÎÕÕÏµ
    if(GetField(pData, VTGpField, 3, MAXFIELD))	  //T
	{
	   GPVTGData.Max = VTGpField[0];	
	}

    //Ë®Æ½ÔË¶¯ËÙ¶È
	if(GetField(pData, VTGpField, 4, MAXFIELD))	  //ËÙ¶È
	{
		if(DataJudge(VTGpField))
		{
	   GPVTGData.speedNot = atof((char *)VTGpField);
		}
	}
	

	//½Ú£¬Knots
    if(GetField(pData, VTGpField, 5, MAXFIELD))	  //T
	{
		 GPVTGData.NotMode = VTGpField[0];			
	}

    //Ë®Æ½ÔË¶¯ËÙ¶È
	if(GetField(pData, VTGpField, 6, MAXFIELD))	  //ËÙ¶È
	{
		if(DataJudge(VTGpField))
		{
	   GPVTGData.speedkm = atof((char *)VTGpField);
		}		
	}


	//-----------------------------
	//¹«Àï/Ê±£¬km/h
	//-----------------------------
    if(GetField(pData, VTGpField, 7, MAXFIELD))	  //T
	{
	   GPVTGData.KmMode = VTGpField[0];		
	}

	//-----------------------------
	// ¶¨Î»Ä£Ê½
	//-----------------------------
	 if(GetField(pData, VTGpField, 8, MAXFIELD))	  //T
	{
	   GPVTGData.PositMode = VTGpField[0];	
	}	   

  //--------------------------------------------------
  // ¾À´í
  //--------------------------------------------------


  if((GPVTGData.Tax!='T')&&(GPVTGData.Max!='M'))
   {   
   	Gps_GError_Flag=1;
   }

  if((GPVTGData.NotMode!='N')&&(GPVTGData.KmMode!='K'))
  {					   
   	Gps_GError_Flag=1;  
  }

  if(Gps_GError_Flag==0)
  {
	  GPVTGData.Count++;    
							
	  VTG_Get_Flag  = 1;						  //

	  Rmc_Vtg_Flag  = 2;              // VTGÄ£Ê½... 
  }

    Gps_GError_Flag  = 0;
}



void ProcessGPZDA(uint8_t *pData)
{

uint8_t     ZDAField[MAXFIELD];
char        ZDApBuff[MAXFIELD];

  //
	// Time
	//
	if(GetField(pData, ZDAField, 0, MAXFIELD))
	{
		// Hour
		ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = '\0';
		GPZDAData.Hour = atoi(ZDApBuff);

		// minute
		ZDApBuff[0] = ZDAField[2];
		ZDApBuff[1] = ZDAField[3];
		ZDApBuff[2] = '\0';
		GPZDAData.Minute = atoi(ZDApBuff);

		// Second
		ZDApBuff[0] = ZDAField[4];
		ZDApBuff[1] = ZDAField[5];
		ZDApBuff[2] = '\0';
		GPZDAData.Second = atoi(ZDApBuff);
		
	 // MSecond
		ZDApBuff[0] = ZDAField[7];
		ZDApBuff[1] = ZDAField[8];
		ZDApBuff[2] = '\0';
		GPZDAData.MSecond = atoi(ZDApBuff);
	}
	
	
	if(GetField(pData, ZDAField, 1, MAXFIELD))
	{

	  ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = '\0';
		GPZDAData.Day = atoi(ZDApBuff);
  }
	
	if(GetField(pData, ZDAField, 2, MAXFIELD))
	{

	  ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = '\0';
		GPZDAData.Month = atoi(ZDApBuff);
  }
	
	if(GetField(pData, ZDAField, 3, MAXFIELD))
	{

	  ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = ZDAField[2];
		ZDApBuff[3] = ZDAField[3];
		ZDApBuff[4] = '\0';
		GPZDAData.Year = atoi(ZDApBuff);
  }
	
	if(GetField(pData, ZDAField, 4, MAXFIELD))
	{

	  ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = '\0';
		GPZDAData.ZHour = atoi(ZDApBuff);
  }
	
	
	if(GetField(pData, ZDAField, 5, MAXFIELD))
	{

	  ZDApBuff[0] = ZDAField[0];
		ZDApBuff[1] = ZDAField[1];
		ZDApBuff[2] = '\0';
		GPZDAData.ZMinute = atoi(ZDApBuff);
  }
		
}

//-------------------------------------
//void ProcessGPGST(uint8_t *pData)
//-------------------------------------
void ProcessGPGST(uint8_t *pData)
{
uint8_t     GSTpField[MAXFIELD];
	
	if(GetField(pData, GSTpField, 1, MAXFIELD))
	{
		GPGSTData.GstRms =atof((char *)GSTpField);	
	}
	
	if(GetField(pData, GSTpField, 5, MAXFIELD))
	{
		GPGSTData.GstDetaLat = atof((char *)GSTpField);		
	}
	else
	{
    GPGSTData.GstDetaLat = 200;
  }
	
	if(GetField(pData, GSTpField, 6, MAXFIELD))
	{
		GPGSTData.GstDetaLon = atof((char *)GSTpField);		
	}
	else
	{
    GPGSTData.GstDetaLon =200;
  }
	
	if(GetField(pData, GSTpField, 7, MAXFIELD))
	{
		GPGSTData.GstDetaAli   = atof((char *)GSTpField);		
	}
	else
	{
    GPGSTData.GstDetaAli =200;
  }

   GST_Get_Flag  = 1;

}




//------------------------------------------
//void ProcessGPGSA(uint8_t *pData)
//------------------------------------------
void ProcessGPGSA(uint8_t *pData)
{
static uint8_t i;
static uint8_t svid,index;
	uint8_t     GSApField[MAXFIELD];
	
  if(GetField(pData, GSApField, 0, MAXFIELD))
	{
		GPGSAData.Mode   = GSApField[0];		
	}
	
	 if(GetField(pData, GSApField, 1, MAXFIELD))
	{
		GPGSAData.FixMode = atoi((char *)GSApField);
	}
	
	for(i=0;i<12;i++)
	{
   if(GetField(pData, GSApField, 2+i, MAXFIELD))
	 {
  	 svid = atoi((char *)GSApField);
	 }
	 else
	 {
    break;
   }
	 
	 if (svid >= MAX_SVID)
				continue;
	 
    index = (svid) / 32;  
	  GPGSAData.SatUse[index]|= 0x01 << ((svid) % 32);
  }
	
	if(GetField(pData, GSApField, 14, MAXFIELD))   //
	{
		GPGSAData.PDOP = atof((char *)GSApField);
	}
	
	if(GetField(pData, GSApField, 15, MAXFIELD))   //
	{
		GPGSAData.HDOP = atof((char *)GSApField);
	}
	
	if(GetField(pData, GSApField, 16, MAXFIELD))  //
	{
		GPGSAData.VDOP = atof((char *)GSApField);
	}
	
	GSA_Get_Flag=1;	

}



//---------------------------------------------
//void ProcessGPGSV(uint8_t *pData)
//---------------------------------------------
void ProcessGPGSV(uint8_t *pData)
{
	static uint8_t i;
	static uint8_t svid,index;
  static uint8_t All,Part;	
	uint8_t     GSVpField[MAXFIELD];
	
	for(i=0;i<4;i++)
	{
	//--------------------------------------------------------------
   if(GetField(pData, GSVpField, 3+4*i, MAXFIELD))
	 {
  	 svid = atoi((char *)GSVpField);
	 }
	 else
	 {
    continue;
   }

	 if (svid >= MAX_SVID)
				continue;
	 
    index = svid / 32;  
	  GPGSVData.SatInView[index]|= 0x01 << (svid % 32);
	 
  //--------------------------------------------------------------
	  if(GetField(pData, GSVpField, 4+4*i, MAXFIELD))
		{
     GPGSVData.SatEl[svid]=atoi((char *)GSVpField);
    }
		else
		{
     continue;;
    }
		
	//--------------------------------------------------------------
	  if(GetField(pData, GSVpField, 5+4*i, MAXFIELD))
		{
     GPGSVData.SatAz[svid]=atoi((char *)GSVpField);
    }
		else
		{
     continue;;
    }
		//--------------------------------------------------------------
	  if(GetField(pData, GSVpField, 6+4*i, MAXFIELD))
		{
     GPGSVData.SatCn0[svid]=atoi((char *)GSVpField);
    }
		else
		{
     continue;
    }	
  }
	
  if(GetField(pData, GSVpField, 0, MAXFIELD))
  {
   All = atoi((char *)GSVpField);
  }
	
	 if(GetField(pData, GSVpField, 1, MAXFIELD))
  {
   Part = atoi((char *)GSVpField);
  }
	
	if(All==Part)
	{
	 //GpsAnalyseFlag=1;        //»ñµÃÕýÈ·µÄGPSÊý¾Ý.......
  }
		 
	GSV_Get_Flag=1;
}




void SendConfigMessage(GPS_CONFIG get_state)
{
  static 	uint8_t  i;
	uint8_t	 Valid_Flag=0;
	static uint8_t  Com_TX_Num=0;
  static   uint8_t   Gps_Command_Data[65]; 
  
	switch(get_state)
	{
		// B5 62 06 00 14 00 01 00 00 00
    // D0 08 00 00 00 C2 01 00 07 00 
		// 02 00 00 00 00 00 BF 78
		case GPS_CONFIG_PRT_SET:  //UBX-CFG-PRT----(1)Target:1-UART1;(2)Protocol in=0+1+2-UBX+NMEA+RTCM; (3)Protocol out:1-NMEA;(4)Baudrate--115200;		
	
			Gps_Command_Data[0] = 0XB5; 																		
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x00; 
			Gps_Command_Data[4] = 0x14; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0x01; 
			Gps_Command_Data[7] = 0x00; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
		
			Gps_Command_Data[10] = 0xD0;
			Gps_Command_Data[11] = 0x08;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x00;
			Gps_Command_Data[15] = 0xC2;
			Gps_Command_Data[16] = 0x01;
			Gps_Command_Data[17] = 0x00;
			Gps_Command_Data[18] = 0x07;
			Gps_Command_Data[19] = 0x00;
			
			Gps_Command_Data[20] = 0x02;
			Gps_Command_Data[21] = 0x00;
			Gps_Command_Data[22] = 0x00;
			Gps_Command_Data[23] = 0x00; 
			Gps_Command_Data[24] = 0x00;
			Gps_Command_Data[25] = 0x00;
			Gps_Command_Data[26] = 0xBF;
			Gps_Command_Data[27] = 0x78; 

	        Valid_Flag  = 1;
			Com_TX_Num  = 28;			
		
		break;
	
	  case GPS_CONFIG_GGA_OPEN:  //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGGA;(2)I2C off;(3)UART1:On,1;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x00; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x00;
			Gps_Command_Data[15] = 0x28;
		  
		 Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
			case GPS_CONFIG_GST_OPEN:  //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGGA;(2)I2C off;(3)UART1:On,1;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x07; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x07;
			Gps_Command_Data[15] = 0x59;
		    Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
    //B5 62 06 01 08 00 F0 02 00 01 00 00 00 00 02 36 
		//B5 62 06 01 08 00 F0 02 00 01 
		//00 00 00 00 02 36
		case GPS_CONFIG_GSA_OPEN: //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGSA;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;

			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x02; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
		
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x02;
			Gps_Command_Data[15] = 0x36;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;					
		break;
   
	 //B5 62 06 01 08 00 F0 03 00 01
   //00 00 00 00 03 3D
	 
	  case GPS_CONFIG_GSV_OPEN:	 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGSV;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
		

			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x03; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
		
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x03;
			Gps_Command_Data[15] = 0x3d;

		
			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
		//B5 62 06 01 08 00 F0 02 00 00
    //00 00 00 00 01 31
		case GPS_CONFIG_GSA_CLOSE: //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGSA;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;

			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x02; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
		
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x01;
			Gps_Command_Data[15] = 0x31;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;					
		break;

   //B5 62 06 01 08 00 F0 03 00 00 
	 //00 00 00 00 02 38
	 
	  case GPS_CONFIG_GSV_CLOSE:	 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGSV;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
		
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x03; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
		
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x02;
			Gps_Command_Data[15] = 0x38;    
  
		
			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
	  case GPS_CONFIG_GLL_CLOSE:	 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxGLL;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x01; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x00;
			Gps_Command_Data[15] = 0x2a;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;

	  case GPS_CONFIG_VTG_CLOSE:	//UBX-CFG-MSG----(1)Massage:F00-00NMEA GxVTG;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x05; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x04;
			Gps_Command_Data[15] = 0x46;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;

	  case GPS_CONFIG_ZDA_OPEN:	//UBX-CFG-MSG----(1)Massage:F00-00NMEA GxVTG;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x08; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x08;
			Gps_Command_Data[15] = 0x60;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
		//B5 62 06 01 08 00 F0 08 00 00 00 00 00 00 07 5B 
		case GPS_CONFIG_ZDA_CLOSE:	//UBX-CFG-MSG----(1)Massage:F00-00NMEA GxVTG;(2)I2C off;(3)UART1:Off;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x08; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x07;
			Gps_Command_Data[15] = 0x5B;

			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
	case GPS_CONFIG_RMC_OPEN:		 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxRMC;(2)I2C off;(3)UART1:ON 1;(4)UART2:Off;(5)SPI:Off;
			Gps_Command_Data[0] = 0xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x01; 
			Gps_Command_Data[4] = 0x08; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xF0; 
			Gps_Command_Data[7] = 0x04; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x01; 
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x04;
			Gps_Command_Data[15] = 0x44;


			Valid_Flag  = 1;
			Com_TX_Num  = 16;			
		break;
		
	//B5 62 06 3E 2C 00 00 00 20 05
  //00 08 10 00 01 00 01 01 01 01 
	//03 00 01 00 01 01 03 08 10 00
  //01 00 01 01 05 00 03 00 01 00
  //01 01 06 08 0E 00 00 00 01 01
  //FF 4D
		case GPS_CONFIG_BD1_OPEN:		 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxRMC;(2)I2C off;(3)UART1:ON 1;(4)UART2:Off;(5)SPI:Off;
			
		 	//B5 62 06 3E 2C 00 00 00 20 05
			Gps_Command_Data[0] = 0xB5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x3E; 
			Gps_Command_Data[4] = 0x2C; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0x00; 
			Gps_Command_Data[7] = 0x00; 
			Gps_Command_Data[8] = 0x20; 
			Gps_Command_Data[9] = 0x05; 
		
			 //00 08 10 00 01 00 01 01 01 01
			Gps_Command_Data[10] = 0x00;		
			Gps_Command_Data[11] = 0x08;
			Gps_Command_Data[12] = 0x10;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x01;
			Gps_Command_Data[15] = 0x00;
			Gps_Command_Data[16] = 0x01;
			Gps_Command_Data[17] = 0x01;
			Gps_Command_Data[18] = 0x01;
			Gps_Command_Data[19] = 0x01;
			
			//03 00 01 00 01 01 03 08 10 00
			Gps_Command_Data[20] = 0x03;		
			Gps_Command_Data[21] = 0x00;
			Gps_Command_Data[22] = 0x01;
			Gps_Command_Data[23] = 0x00;
			Gps_Command_Data[24] = 0x01;
			Gps_Command_Data[25] = 0x01;
			Gps_Command_Data[26] = 0x03;
			Gps_Command_Data[27] = 0x08;
			Gps_Command_Data[28] = 0x10;
			Gps_Command_Data[29] = 0x00;
			
			//01 00 01 01 05 00 03 00 01 00
			Gps_Command_Data[30] = 0x01;		
			Gps_Command_Data[31] = 0x00;
			Gps_Command_Data[32] = 0x01;
			Gps_Command_Data[33] = 0x01;
			Gps_Command_Data[34] = 0x05;
			Gps_Command_Data[35] = 0x00;
			Gps_Command_Data[36] = 0x03;
			Gps_Command_Data[37] = 0x00;
			Gps_Command_Data[38] = 0x01;
			Gps_Command_Data[39] = 0x00;

		 //01 01 06 08 0E 00 00 00 01 01
			Gps_Command_Data[40] = 0x01;		
			Gps_Command_Data[41] = 0x01;
			Gps_Command_Data[42] = 0x06;
			Gps_Command_Data[43] = 0x08;
			Gps_Command_Data[44] = 0x0E;
			Gps_Command_Data[45] = 0x00;
			Gps_Command_Data[46] = 0x00;
			Gps_Command_Data[47] = 0x00;
			Gps_Command_Data[48] = 0x01;
			Gps_Command_Data[49] = 0x01;
			
			//FF 4D
			Gps_Command_Data[50] = 0xFF;		
			Gps_Command_Data[51] = 0x4D;
			
			Valid_Flag  = 1;
			Com_TX_Num  = 52;			
		break;

//B5 62 06 17 14 00 00 40 00 02
//00 00 00 00 01 00 00 01 00 00 
//00 00 00 00 00 00 75 50
	   case GPS_CONFIG_BD2_OPEN:		 //UBX-CFG-MSG----(1)Massage:F00-00NMEA GxRMC;(2)I2C off;(3)UART1:ON 1;(4)UART2:Off;(5)SPI:Off;
			 
			Gps_Command_Data[0] = 0xB5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x17; 
			Gps_Command_Data[4] = 0x14; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0x00; 
			Gps_Command_Data[7] = 0x40; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x02; 
		 	 
		 //00 00 00 00 01 00 00 01 00 00 
		 	Gps_Command_Data[10] = 0x00; 
			Gps_Command_Data[11] = 0x00; 
			Gps_Command_Data[12] = 0x00; 
		 	Gps_Command_Data[13] = 0x00; 
		 	Gps_Command_Data[14] = 0x01; 
		 	Gps_Command_Data[15] = 0X00; 
			Gps_Command_Data[16] = 0x00; 
			Gps_Command_Data[17] = 0x01; 
		 	Gps_Command_Data[18] = 0x00; 
			Gps_Command_Data[19] = 0x00; 
			
			//00 00 00 00 00 00 75 50
			Gps_Command_Data[20] = 0X00; 
			Gps_Command_Data[21] = 0x00; 
			Gps_Command_Data[22] = 0x00; 
		 	Gps_Command_Data[23] = 0x00; 
		 	Gps_Command_Data[24] = 0x00; 
		 	Gps_Command_Data[25] = 0X00; 
			Gps_Command_Data[26] = 0x75; 
			Gps_Command_Data[27] = 0x50; 
			
		 
		 	Valid_Flag  = 1;
			Com_TX_Num  = 28;		
		 break;

// B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 17 31 BF 
		case GPS_CONFIG_SAVE: //UBX-CFG-CFG----(1)Save current conffiguration;(2)Devices:0-BBR,1-FLASH,2-I2C-EEPROM
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x09; 
			Gps_Command_Data[4] = 0x0D; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0x00; 
			Gps_Command_Data[7] = 0x00; 
			Gps_Command_Data[8] = 0x00; 
			Gps_Command_Data[9] = 0x00; 
			Gps_Command_Data[10] = 0xff;
			Gps_Command_Data[11] = 0xff;
			Gps_Command_Data[12] = 0x00;
			Gps_Command_Data[13] = 0x00;
			Gps_Command_Data[14] = 0x00;
			Gps_Command_Data[15] = 0x00;
			Gps_Command_Data[16] = 0x00;
			Gps_Command_Data[17] = 0x00;
			Gps_Command_Data[18] = 0x17;
			Gps_Command_Data[19] = 0x31;
			Gps_Command_Data[20] = 0xBF;

			Valid_Flag  = 1;
			Com_TX_Num  = 21;			
		break;

				case GPS_CONFIG_RATE: //UBX-CFG-RATE----(1)1-GPS Time;(2)Measurement Period:250(ms)(3)Measurement Frequency:4;(4)Navigation Rate:1 cyc(5)Navigation Frequency 4hz
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x08; 
			Gps_Command_Data[4] = 0x06; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0xC8; 
			Gps_Command_Data[7] = 0x00; 												
			Gps_Command_Data[8] = 0x01; 
			Gps_Command_Data[9] = 0x00; 
			Gps_Command_Data[10] = 0x01;
			Gps_Command_Data[11] = 0x00;
			Gps_Command_Data[12] = 0xDE;
			Gps_Command_Data[13] = 0x6A;

			Valid_Flag  = 1;
			Com_TX_Num  = 14;			
		break;
case GPS_CONFIG_GL_OPEN: //UBX-CFG-RATE----(1)1-GPS Time;(2)Measurement Period:250(ms)(3)Measurement Frequency:4;(4)Navigation Rate:1 cyc(5)Navigation Frequency 4hz
			Gps_Command_Data[0] = 0Xb5; 
			Gps_Command_Data[1] = 0x62; 
			Gps_Command_Data[2] = 0x06; 
			Gps_Command_Data[3] = 0x3E; 
			Gps_Command_Data[4] = 0x34; 
			Gps_Command_Data[5] = 0x00; 
			Gps_Command_Data[6] = 0x00; 
			Gps_Command_Data[7] = 0x00; 												
			Gps_Command_Data[8] = 0x20; 
			Gps_Command_Data[9] = 0x06; 
				
			Gps_Command_Data[10] = 0x00;
			Gps_Command_Data[11] = 0x08;
			Gps_Command_Data[12] = 0x10;
			Gps_Command_Data[13] = 0x00;
      Gps_Command_Data[14] = 0x01; 
			Gps_Command_Data[15] = 0x00; 
			Gps_Command_Data[16] = 0x01;
			Gps_Command_Data[17] = 0x01;
			Gps_Command_Data[18] = 0x01;
			Gps_Command_Data[19] = 0x01;

			Gps_Command_Data[20] = 0X03; 
			Gps_Command_Data[21] = 0x00; 
			Gps_Command_Data[22] = 0x00; 
			Gps_Command_Data[23] = 0x00; 
			Gps_Command_Data[24] = 0x01; 
			Gps_Command_Data[25] = 0x01; 
			Gps_Command_Data[26] = 0x03; 
			Gps_Command_Data[27] = 0x08; 												
			Gps_Command_Data[28] = 0x10; 
			Gps_Command_Data[29] = 0x00; 
			
			Gps_Command_Data[30] = 0x00; 
			Gps_Command_Data[31] = 0x00; 												
			Gps_Command_Data[32] = 0x01; 
			Gps_Command_Data[33] = 0x01; 
			Gps_Command_Data[34] = 0x04; 
			Gps_Command_Data[35] = 0x00; 
			Gps_Command_Data[36] = 0x08; 
			Gps_Command_Data[37] = 0x00; 												
			Gps_Command_Data[38] = 0x00; 
			Gps_Command_Data[39] = 0x00;
			
			Gps_Command_Data[40] = 0X01; 
			Gps_Command_Data[41] = 0x01; 
			Gps_Command_Data[42] = 0x05; 
			Gps_Command_Data[43] = 0x00; 
			Gps_Command_Data[44] = 0x03; 
			Gps_Command_Data[45] = 0x00; 
			Gps_Command_Data[46] = 0x01; 
			Gps_Command_Data[47] = 0x00; 												
			Gps_Command_Data[48] = 0x01; 
			Gps_Command_Data[49] = 0x01; 
			
			Gps_Command_Data[50] = 0x06; 
			Gps_Command_Data[51] = 0x08; 												
			Gps_Command_Data[52] = 0x0e; 
			Gps_Command_Data[53] = 0x00; 
			Gps_Command_Data[54] = 0x01; 
			Gps_Command_Data[55] = 0x00; 
			Gps_Command_Data[56] = 0x01; 
			Gps_Command_Data[57] = 0x01; 												
			Gps_Command_Data[58] = 0x15; 
			Gps_Command_Data[59] = 0xd5;
			
						
			
			Valid_Flag  = 1;
			Com_TX_Num  = 60;			
		break;	

	default:  break;
	}


	  for(i=0;i<Com_TX_Num;i++)
		{
		 USART_SendData(USART2,Gps_Command_Data[i]);
		 while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
	  }

	 Com_TX_Num=0;	 

}


//---------------------------------
void GPS_Rate_Config(void)
{
  	USART2_BodRate=9600;
	  USART_Configuration();
    GpsBautConfig();

	  USART2_BodRate=115200;
	  USART_Configuration();

}

void GpsBautConfig(void)
{
	uint8_t dtime=50;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//¿ªÆô½ÓÊÜÖÐ¶Ï£¬¿ªÊ¼½ÓÊÜÊý¾Ý
	
  SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte,4hz
	delay_ms(dtime);
	SendConfigMessage(GPS_CONFIG_PRT_SET);  	  //ÉèÖÃ¶Ë¿Ú115200    420*4/9600=0.175
	delay_ms(dtime);
}

void Flag_Clear(void)
{
		GGA_Config_flag = 0;
		GST_Config_flag = 0;
		GSA_Config_flag = 0;
		GSV_Config_flag = 0;
		GLL_Config_flag = 0;
		VTG_Config_flag = 0;
		ZDA_Config_flag = 0;
		RMC_Config_flag = 0;
	  GL_Config_flag  = 0;
    BD1_Config_flag = 0;
    BD2_Config_flag = 0;
}

//---------------------------------
//void GpsGsvClose(void)
//---------------------------------
//---------------------------------
//void GpsGsvClose(void)
//---------------------------------
void GpsGsvClose(void)
{
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte,4hz
		if(GGA_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			//SerialPutString("GGA OPEN SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("GGA OPEN FAILED!\r\n");
		}
	}
	
	  while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSA_CLOSE); 	  //¹Ø±ÕGSA
			if(GSA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSA CLOSE SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSA CLOSE FAILED!\r\n");
			}
		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSV_CLOSE); 	  //¹Ø±ÕGSA
			if(GSV_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSV CLOSE SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSV CLOSE FAILED!\r\n");
			}
		}
}


//-------------------------
//void GpsGsvOpen(void)
//-------------------------
void GpsGsvOpen(void)
{
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte,4hz
		if(GGA_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			//SerialPutString("GGA OPEN SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("GGA OPEN FAILED!\r\n");
		}
	}
	
	  while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSA_OPEN); 	  //¹Ø±ÕGSA
			if(GSA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSA OPEN SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSA OPEN FAILED!\r\n");
			}
		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSV_OPEN); 	  //¹Ø±ÕGSA
			if(GSV_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSV OPEN SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSV OPEN FAILED!\r\n");
			}
		}
}

//-------------------------
//void GpsBDOpen(void)
//-------------------------
void GpsBDOpen(void)
{
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte,4hz
			if(GGA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				//SerialPutString("GGA OPEN SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GGA OPEN FAILED!\r\n");
			}
		}
		
		SendConfigMessage(GPS_CONFIG_BD1_OPEN); 	  //´ò¿ªBD1
		delay_ms(50);
//	  while(1)
//		{
//			timer_start_flag = 1;
//			
//			SendConfigMessage(GPS_CONFIG_BD1_OPEN); 	  //´ò¿ªBD1
//			if(BD1_Config_flag)
//			{
//				timer_start_flag = 0;
//				ttime = 0;
//				SerialPutString("BD1 OPEN SUCCESSFULLY!\r\n");
//				BD1_Config_flag = 0;
//				break;
//			}
//			
//			if(ttime>1000)
//			{
//				ttime = 0;
//				SerialPutString("BD1 OPEN FAILED!\r\n");
//			}
//		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_BD2_OPEN); 	  //´ò¿ªBD1
			if(BD2_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("BD2 OPEN SUCCESSFULLY!\r\n");
				BD2_Config_flag = 0;
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("BD2 OPEN FAILED!\r\n");
			}
		}
}


//----------------------------
//void GpsGLOpen(void)
//----------------------------
void GpsGLOpen(void)
{
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte,4hz
			if(GGA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				//SerialPutString("GGA OPEN SUCCESSFULLY!\r\n");
				BD1_Config_flag = 0;
				GL_Config_flag = 0;
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GGA OPEN FAILED!\r\n");
			}
		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GL_OPEN); 	  //
			if(GL_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GL OPEN SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GL OPEN FAILED!\r\n");
			}
		}
  
	BD_GGet_Flag=0; 
}
//-----------------------------------
//void GpsConfig2(void)
//-----------------------------------
void GpsConfig2(void)
{
	uint8_t dtime=5;
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//¿ªÆô½ÓÊÜÖÐ¶Ï£¬¿ªÊ¼½ÓÊÜÊý¾Ý
	
	SerialPutString("Configuration:\r\n");

	SerialPutString("-----------------------------------\r\n");

	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_GGA_OPEN);		  //´ò¿ªGGA	 42Byte
		if(GGA_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("GGA OPEN  SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("GGA OPEN FAILED!\r\n");
			GPS_Rate_Config();                       //´ÓÐÂÅäÖÃ²¨ÌØÂÊ
		}
	}
	
		while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_RMC_OPEN); 	  //´ò¿ªRMC
		if(RMC_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("RMC OPEN  SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("RMC OPEN FAILED!\r\n");
			GPS_Rate_Config();                       //´ÓÐÂÅäÖÃ²¨ÌØÂÊ
		}
	}
	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_GST_OPEN);		  //´ò¿ªGST	 42Byte
		if(GST_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("GST OPEN  SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("GST OPEN FAILED!\r\n");
			GPS_Rate_Config();                       //´ÓÐÂÅäÖÃ²¨ÌØÂÊ
		}
	}
	
	//BD_DGet_Flag=0;
	if(BD_DGet_Flag)	
	{
		SendConfigMessage(GPS_CONFIG_BD1_OPEN); 	  //´ò¿ªBD1
		delay_ms(50);
		
//		while(1)
//		{
//			timer_start_flag = 1;
//			
//			SendConfigMessage(GPS_CONFIG_BD1_OPEN); 	  //´ò¿ªBD1
//			if(BD1_Config_flag)
//			{
//				timer_start_flag = 0;
//				ttime = 0;
//				SerialPutString("BD1 OPEN  SUCCESSFULLY!\r\n");
//				BD1_Config_flag = 0;
//				break;
//			}
//			
//			if(ttime>1000)
//			{
//				ttime = 0;
//				SerialPutString("BD1 OPEN FAILED!\r\n");
//			}
//		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_BD2_OPEN); 	  //´ò¿ªBD1
			if(BD2_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("BD2 OPEN  SUCCESSFULLY!\r\n");
				BD2_Config_flag = 0;
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("BD2 OPEN FAILED!\r\n");
			}
		}
		
		BD_GGet_Flag=1; 
	}
	else
	{
	  while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GL_OPEN); 	  //
			if(GL_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GL  OPEN  SUCCESSFULLY!\r\n");
				BD1_Config_flag = 0;
				GL_Config_flag = 0;
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GL OPEN FAILED!\r\n");
			}
		}
		

	  BD_GGet_Flag=0; 
	}
	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_GLL_CLOSE); 	  //¹Ø±ÕGLL
		if(GLL_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("GLL CLOSE SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("GLL CLOSE FAILED!\r\n");
		}
	}
	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_VTG_CLOSE); 	  //¹Ø±ÕVTG
		if(VTG_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("VTG CLOSE SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
		
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("VTG CLOSE FAILED!\r\n");
		}
	}
	

  
	
	if(GSV_DGet_Flag)
	{
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSA_OPEN); 	  //¹Ø±ÕGSA
			if(GSA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSA OPEN  SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSA OPEN FAILED!\r\n");
			}
		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSV_OPEN); 	  //¹Ø±ÕGSA
			if(GSV_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSV OPEN  SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSV OPEN FAILED!\r\n");
			}
		}
	}
	else
	{
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSA_CLOSE); 	  //¹Ø±ÕGSA
			if(GSA_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSA CLOSE SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSA CLOSE FAILED!\r\n");
			}
		}
		
		while(1)
		{
			timer_start_flag = 1;
			
			SendConfigMessage(GPS_CONFIG_GSV_CLOSE); 	  //¹Ø±ÕGSA
			if(GSV_Config_flag)
			{
				timer_start_flag = 0;
				ttime = 0;
				SerialPutString("GSV CLOSE SUCCESSFULLY!\r\n");
				Flag_Clear();
				break;
			}
			
			if(ttime>1000)
			{
				ttime = 0;
				SerialPutString("GSV CLOSE FAILED!\r\n");
			}
		}
	}
	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_ZDA_CLOSE); 	  //¹Ø±ÕZDA	
		if(ZDA_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("ZDA CLOSE SUCCESSFULLY!\r\n");
			Flag_Clear();
			break;
		}
			
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("ZDA CLOSE FAILED!\r\n");
		}
	}
	
	while(1)
	{
		timer_start_flag = 1;
		
		SendConfigMessage(GPS_CONFIG_RATE);	    	  //ÉèÖÃRATE 200ms
		if(RATE_Config_flag)
		{
			timer_start_flag = 0;
			ttime = 0;
			SerialPutString("RATE CONF SUCCESSFULLY!\r\n");
			RATE_Config_flag = 0;
			break;
		}
			
		if(ttime>1000)
		{
			ttime = 0;
			SerialPutString("RATE CONFIG FAILED!\r\n");
		}
	}
	
	SerialPutString("-----------------------------------\r\n");
}









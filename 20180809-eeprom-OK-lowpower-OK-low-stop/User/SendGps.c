#include "SendGPS.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "hw_config.h"

#include "GetAcc.h"
#include "GetGyr.h"
#include "GetIMU.h"
#include "GetGPS.h"
#include "GetNass.h"
#include "commdata.h"
#include  "EncryptID.h"
#include "stmflash.h"

#include "GINavMain.h"
#include "GlobalVars.h"
#include "DataProc.h"

//************************************************************
//            GPS编码
//************************************************************



//------------------------------------------------
//void ProcessLSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------
uint8_t ProcessLSFTOA(float GGAIData,uint8_t bit)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;    
 static uint8_t ZF_Flag;  
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;


 if(GGAIData>=0)  
 {
  ZF_Flag=1;
  
 }
 else
 {
  ZF_Flag=0;  
 }

 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
  if(abs(Angle)>=100)
 {
  Degree=3;
 }
 else if(abs(Angle)>=10)
 {
  Degree=2;
 }
 else if(abs(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
  if(ZF_Flag==0)
  {
   Angle    = - Angle;
   AngleDot = - AngleDot;
  }
 //--------------------------------------------------------------
  if(Degree==3)                                                //132
  {
	 GGB_Data[0] = (Angle)/100;	                                //百位    
   GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //十位        
	 GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //个数        		    

	 ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] = GGB_Data[1] + 0x30;
	 GGA_Data[3] = GGB_Data[2] + 0x30;
	 GGA_Data[4] = '.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[5+i] =GGS_Data[i];
	 }

	 AVRNum = 5+bit;

	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                          //13
	{
	 GGB_Data[0] = (Angle)/10;	                                //十位    
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //个位        	

	 ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] = GGB_Data[1] + 0x30;	 
	 GGA_Data[3] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[4+i] =GGS_Data[i];
	 }

	 AVRNum = 4+bit;
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //个位

	 ProcessSSFTOA(AngleDot,bit);

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3 + bit;
	}
   //-----------------------------
	else
	{
     GGB_Data[0] = 0;

	 ProcessSSFTOA(AngleDot,bit);

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;	 
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3+bit;
	}
  
   return AVRNum;										
}


//------------------------------------------------
//void ProcessLSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------
uint8_t ProcessPSFTOA(float GGAIData,uint8_t bit)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;    
 static uint8_t ZF_Flag;  
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;


 if(GGAIData>=0)  
 {
  ZF_Flag=1;
  
 }
 else
 {
  ZF_Flag=0;  
 }

 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
  if(abs(Angle)>=100)
 {
  Degree=3;
 }
 else if(abs(Angle)>=10)
 {
  Degree=2;
 }
 else if(abs(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
  if(ZF_Flag==0)
  {
   Angle    = - Angle;
   AngleDot = - AngleDot;
  }
 //--------------------------------------------------------------
  if(Degree==3)                                                //132
  {
	 GGB_Data[0] = (Angle)/100;	                                //百位    
   GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //十位        
	 GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //个数        		    

	 ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
	  GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] = GGB_Data[1] + 0x30;
	  GGA_Data[2] = GGB_Data[2] + 0x30;
	  GGA_Data[3] = '.';

	  for(i=0;i<bit;i++)
	  {
	  GGA_Data[4+i] =GGS_Data[i];
	  }
	  AVRNum = 4+bit;
		
	 }
	 else
	 {
	  GGA_Data[0]='-';	 

    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] = GGB_Data[1] + 0x30;
	  GGA_Data[3] = GGB_Data[2] + 0x30;
	  GGA_Data[4] = '.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[5+i] =GGS_Data[i];
	  }
	  AVRNum = 5+bit;
   }
	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                          //13
	{
	 GGB_Data[0] = (Angle)/10;	                                //十位    
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //个位        	

	 ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] = GGB_Data[1] + 0x30;	 
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }
	   AVRNum = 3+bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 
    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] = GGB_Data[1] + 0x30;	 
	  GGA_Data[3] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[4+i] =GGS_Data[i];
	  }
	   AVRNum = 4+bit;
   }
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //个位

	  ProcessSSFTOA(AngleDot,bit);

	  if(ZF_Flag)
	  {
     GGA_Data[0] = GGB_Data[0] + 0x30;
	   GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }
	  AVRNum = 2 + bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }
	  AVRNum = 3 + bit;
   }
	}
   //-----------------------------
	else
	{
   GGB_Data[0] = 0;

	 ProcessSSFTOA(AngleDot,bit);

	 if(ZF_Flag)
	 {
    GGA_Data[0] = GGB_Data[0] + 0x30;	 
	  GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }

	  AVRNum = 2+bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
	  GGA_Data[1] = GGB_Data[0] + 0x30;	 
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }

	  AVRNum = 3+bit;
	 }
  }
   return AVRNum;										
}

//------------------------------------------------
//void ProcessLatToA(double GGAIData)
//------------------------------------------------
void ProcessLatToA(double GGAIData)
{
static   uint8_t  i;
static   uint8_t  Latitude;

static  double  DLatitudePart;
static  uint64_t LatitudePart;
	

   Latitude    = GGAIData;
   												     	   //31.2754477==>31+0.2754477*60==>31+16.52686 ==> 3116.52686;

    GGA_Data[0] = Latitude/10 ;					   //d 3
    GGA_Data[1] = Latitude    - ((uint16_t)GGA_Data[0])*10 ;      //d 1

    DLatitudePart= (GGAIData - Latitude)*60.0;
    LatitudePart = (DLatitudePart)*100000.0;      //degree;
   
    GGA_Data[2] = LatitudePart/1000000 ;                                         //m
    GGA_Data[3] = LatitudePart/100000  - GGA_Data[2]*10;                       //m
    GGA_Data[4] = 0x2E;									                              //.
    GGA_Data[5] = LatitudePart/10000   - ((uint32_t)GGA_Data[2])*100    - ((uint32_t)GGA_Data[3])*10 ;     //m
    GGA_Data[6] = LatitudePart/1000    - ((uint32_t)GGA_Data[2])*1000   - ((uint32_t)GGA_Data[3])*100    - ((uint32_t)GGA_Data[5])*10;     //m
    GGA_Data[7] = LatitudePart/100     - ((uint32_t)GGA_Data[2])*10000  - ((uint32_t)GGA_Data[3])*1000   - ((uint32_t)GGA_Data[5])*100   - ((uint32_t)GGA_Data[6])*10;     //m
    GGA_Data[8] = LatitudePart/10      - ((uint32_t)GGA_Data[2])*100000 - ((uint32_t)GGA_Data[3])*10000  - ((uint32_t)GGA_Data[5])*1000  - ((uint32_t)GGA_Data[6])*100  -  ((uint32_t)GGA_Data[7])*10;     //m
    GGA_Data[9] = LatitudePart         - ((uint32_t)GGA_Data[2])*1000000- ((uint32_t)GGA_Data[3])*100000 - ((uint32_t)GGA_Data[5])*10000 - ((uint32_t)GGA_Data[6])*1000 -  ((uint32_t)GGA_Data[7])*100 -  ((uint32_t)GGA_Data[8])*10;     //m

		 
    for (i=0;i<4;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }

    for (i=5;i<10;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }
   
}
//-------------------------------------------------
//void ProcessSSFTOA(float GGAIData,uint8_t bit)
//-------------------------------------------------
void ProcessSSFTOA(double GGAIData,uint8_t bit)
{
 static double  AngDot;
 static uint8_t i;
 static uint64_t Anglel;
	
   AngDot= GGAIData;	
	
	
   if(bit==11)			       //bit==11
	 {
	  Anglel=AngDot*100000000000;
		 
    GGS_Data[0] = (Anglel)/10000000000;	                              //千位    	
    GGS_Data[1] = (Anglel)/1000000000  - ((uint32_t)GGS_Data[0]*10);     //百位        				                             //.
	  GGS_Data[2] = (Anglel)/100000000   - ((uint32_t)GGS_Data[0]*100)         - ((uint32_t)GGS_Data[1]*10) ;  //十数        	
	  GGS_Data[3] = (Anglel)/10000000    - ((uint32_t)GGS_Data[0]*1000)        - ((uint32_t)GGS_Data[1]*100)        - ((uint32_t)GGS_Data[2]*10) ;  //十数        	
		GGS_Data[4] = (Anglel)/1000000     - ((uint32_t)GGS_Data[0]*10000)       - ((uint32_t)GGS_Data[1]*1000)       - ((uint32_t)GGS_Data[2]*100)       - ((uint32_t)GGS_Data[3]*10) ;  //十数        	
    GGS_Data[5] = (Anglel)/100000      - ((uint32_t)GGS_Data[0]*100000)      - ((uint32_t)GGS_Data[1]*10000)      - ((uint32_t)GGS_Data[2]*1000)      - ((uint32_t)GGS_Data[3]*100)     - ((uint32_t)GGS_Data[4]*10);  //十数        	
		GGS_Data[6] = (Anglel)/10000       - ((uint32_t)GGS_Data[0]*1000000)     - ((uint32_t)GGS_Data[1]*100000)     - ((uint32_t)GGS_Data[2]*10000)     - ((uint32_t)GGS_Data[3]*1000)    - ((uint32_t)GGS_Data[4]*100)     - ((uint32_t)GGS_Data[5]*10);  //十数        	
		GGS_Data[7] = (Anglel)/1000        - ((uint32_t)GGS_Data[0]*10000000)    - ((uint32_t)GGS_Data[1]*1000000)    - ((uint32_t)GGS_Data[2]*100000)    - ((uint32_t)GGS_Data[3]*10000)   - ((uint32_t)GGS_Data[4]*1000)    - ((uint32_t)GGS_Data[5]*100)    - ((uint32_t)GGS_Data[6]*10);  //十数        	 
		GGS_Data[8] = (Anglel)/100         - ((uint32_t)GGS_Data[0]*100000000)   - ((uint32_t)GGS_Data[1]*10000000)   - ((uint32_t)GGS_Data[2]*1000000)   - ((uint32_t)GGS_Data[3]*100000)  - ((uint32_t)GGS_Data[4]*10000)   - ((uint32_t)GGS_Data[5]*1000)   - ((uint32_t)GGS_Data[6]*100)   - ((uint32_t)GGS_Data[7]*10);  //十数        	  
		GGS_Data[9] = (Anglel)/10          - ((uint64_t)GGS_Data[0]*1000000000)  - ((uint64_t)GGS_Data[1]*100000000)  - ((uint32_t)GGS_Data[2]*10000000)  - ((uint32_t)GGS_Data[3]*1000000) - ((uint32_t)GGS_Data[4]*100000)  - ((uint32_t)GGS_Data[5]*10000)  - ((uint32_t)GGS_Data[6]*1000)  - ((uint32_t)GGS_Data[7]*100)  - ((uint32_t)GGS_Data[8]*10);  //十数        	  
		GGS_Data[10]= (Anglel)             - ((uint64_t)GGS_Data[0]*10000000000) - ((uint64_t)GGS_Data[1]*1000000000) - ((uint32_t)GGS_Data[2]*100000000) - ((uint32_t)GGS_Data[3]*10000000)- ((uint32_t)GGS_Data[4]*1000000) - ((uint32_t)GGS_Data[5]*100000) - ((uint32_t)GGS_Data[6]*10000) - ((uint32_t)GGS_Data[7]*1000) - ((uint32_t)GGS_Data[8]*100)- ((uint32_t)GGS_Data[9]*10);  //十数        	  
		 
		for(i=0;i<11;i++)
	  {
	   if(GGS_Data[i]==0xFF)		
			GGS_Data[i]=0;
		
     GGS_Data[i] = GGS_Data[i] + 0x30;   
		}
   }
	 if(bit==9)			       //bit==11
	 {
	  Anglel=AngDot*1000000000;
		 
    GGS_Data[0] = (Anglel)/100000000;	                              //千位    	
    GGS_Data[1] = (Anglel)/10000000  - ((uint32_t)GGS_Data[0]*10);     //百位        				                             //.
	  GGS_Data[2] = (Anglel)/1000000   - ((uint32_t)GGS_Data[0]*100)         - ((uint32_t)GGS_Data[1]*10) ;  //十数        	
	  GGS_Data[3] = (Anglel)/100000    - ((uint32_t)GGS_Data[0]*1000)        - ((uint32_t)GGS_Data[1]*100)        - ((uint32_t)GGS_Data[2]*10) ;  //十数        	
		GGS_Data[4] = (Anglel)/10000     - ((uint32_t)GGS_Data[0]*10000)       - ((uint32_t)GGS_Data[1]*1000)       - ((uint32_t)GGS_Data[2]*100)       - ((uint32_t)GGS_Data[3]*10) ;  //十数        	
    GGS_Data[5] = (Anglel)/1000      - ((uint32_t)GGS_Data[0]*100000)      - ((uint32_t)GGS_Data[1]*10000)      - ((uint32_t)GGS_Data[2]*1000)      - ((uint32_t)GGS_Data[3]*100)     - ((uint32_t)GGS_Data[4]*10);  //十数        	
		GGS_Data[6] = (Anglel)/100       - ((uint32_t)GGS_Data[0]*1000000)     - ((uint32_t)GGS_Data[1]*100000)     - ((uint32_t)GGS_Data[2]*10000)     - ((uint32_t)GGS_Data[3]*1000)    - ((uint32_t)GGS_Data[4]*100)     - ((uint32_t)GGS_Data[5]*10);  //十数        	
		GGS_Data[7] = (Anglel)/10        - ((uint32_t)GGS_Data[0]*10000000)    - ((uint32_t)GGS_Data[1]*1000000)    - ((uint32_t)GGS_Data[2]*100000)    - ((uint32_t)GGS_Data[3]*10000)   - ((uint32_t)GGS_Data[4]*1000)    - ((uint32_t)GGS_Data[5]*100)    - ((uint32_t)GGS_Data[6]*10);  //十数        	 
		GGS_Data[8] = (Anglel)           - ((uint32_t)GGS_Data[0]*100000000)   - ((uint32_t)GGS_Data[1]*10000000)   - ((uint32_t)GGS_Data[2]*1000000)   - ((uint32_t)GGS_Data[3]*100000)  - ((uint32_t)GGS_Data[4]*10000)   - ((uint32_t)GGS_Data[5]*1000)   - ((uint32_t)GGS_Data[6]*100)   - ((uint32_t)GGS_Data[7]*10);  //十数        	  
		 
		for(i=0;i<9;i++)
	  {
	   GGS_Data[i] = GGS_Data[i] + 0x30;   
		}
   }
   else if(bit==4)			       //bit==4
   {
    GGS_Data[0] = (AngDot*10000)/1000;	                              //千位    	
    GGS_Data[1] = (AngDot*10000)/100  - ((uint16_t)GGS_Data[0])*10;     //百位        				                             //.
	  GGS_Data[2] = (AngDot*10000)/10   - ((uint16_t)GGS_Data[0])*100 - ((uint16_t)GGS_Data[1]*10) ;  //十数        	
	  GGS_Data[3] = (AngDot*10000)      - ((uint16_t)GGS_Data[0])*1000- ((uint16_t)GGS_Data[1]*100)- ((uint16_t)GGS_Data[2]*10) ;  //十数        	

    GGS_Data[0] = GGS_Data[0] + 0x30;   
    GGS_Data[1] = GGS_Data[1] + 0x30;   
    GGS_Data[2] = GGS_Data[2] + 0x30;       
	  GGS_Data[3] = GGS_Data[3] + 0x30;  
   }
   else	                        //bit==3
   {
    GGS_Data[0] = (AngDot*1000)/100;	                              //千位    	
    GGS_Data[1] = (AngDot*1000)/10  - ((uint16_t)GGS_Data[0]*10);     //百位        				                             //.
	  GGS_Data[2] = (AngDot*1000)     - ((uint16_t)GGS_Data[0])*100-((uint16_t)GGS_Data[1]*10);
	

    GGS_Data[0] = GGS_Data[0] + 0x30;   
    GGS_Data[1] = GGS_Data[1] + 0x30;   
    GGS_Data[2] = GGS_Data[2] + 0x30;       	  
   }													  
}

//-------------------------------------------------------
//uint8_t ProcessZLSFTOA(float GGAIData,uint8_t bit)
//-------------------------------------------------------
uint8_t ProcessZLSFTOA(float GGAIData,uint8_t bit)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;     
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;



 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
 if(abs(Angle)>=1000)
 {
  Degree=4;
 }
  if(abs(Angle)>=100)
 {
  Degree=3;
 }
 else if(abs(Angle)>=10)
 {
  Degree=2;
 }
 else if(abs(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }


 //--------------------------------------------------------------
  if(Degree==4)                                                //132
  {
	 GGB_Data[0] = (Angle)/1000;	                                //千位    
   GGB_Data[1] = (Angle)/100    - ((uint16_t)GGB_Data[0])*10;   //百位     
	 GGB_Data[2] = (Angle)/10     - ((uint16_t)GGB_Data[0])*100 -  ((uint16_t)GGB_Data[1]*10) ;  //十位
   GGB_Data[3] = (Angle)        - ((uint16_t)GGB_Data[0])*1000 - ((uint16_t)GGB_Data[1]*100) -GGB_Data[2]*10;  //个		

	 ProcessSSFTOA(AngleDot,bit);							   

	
   GGA_Data[0] = GGB_Data[0] + 0x30;
	 GGA_Data[1] = GGB_Data[1] + 0x30;
	 GGA_Data[2] = GGB_Data[2] + 0x30;
	 GGA_Data[3] = GGB_Data[3] + 0x30;
	 GGA_Data[4] = '.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[5+i] =GGS_Data[i];
	 }

	 AVRNum = 5+bit;

	}
	
  else if(Degree==3)                                                //132
  {
	 GGB_Data[0] = (Angle)/100;	                                //百位    
   GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //十位        
	 GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //个数        		    

	 ProcessSSFTOA(AngleDot,bit);							   

	
   GGA_Data[0] = GGB_Data[0] + 0x30;
	 GGA_Data[1] = GGB_Data[1] + 0x30;
	 GGA_Data[2] = GGB_Data[2] + 0x30;
	 GGA_Data[3] = '.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[4+i] =GGS_Data[i];
	 }

	 AVRNum = 4+bit;

	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                          //13
	{
	 GGB_Data[0] = (Angle)/10;	                                //十位    
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //个位        	

	 ProcessSSFTOA(AngleDot,bit);							   

	 
   GGA_Data[0] = GGB_Data[0] + 0x30;
	 GGA_Data[1] = GGB_Data[1] + 0x30;	 
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3+bit;
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //个位

	  ProcessSSFTOA(AngleDot,bit);


    GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }

	  AVRNum = 2 + bit;
	}
   //-----------------------------
	else
	{
    GGB_Data[0] = 0;

	  ProcessSSFTOA(AngleDot,bit);	 

    GGA_Data[0] = GGB_Data[0] + 0x30;	 
	  GGA_Data[1] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[2+i] =GGS_Data[i];
	 }

	 AVRNum = 2+bit;
	}
  
   return AVRNum;										
}
//------------------------------------------------
//void ProcessLongToA(double GGAIData)
//------------------------------------------------
void ProcessLongToA(double GGAIData)
{
static   uint8_t  i,Longitude;
static   uint64_t LongitudePart;

   Longitude   = GGAIData;			     	   //121.452914===121+0.452914*60 =121+27.17484==12127.17484;

   GGA_Data[0] = Longitude/100;					                       //d 1
   GGA_Data[1] = Longitude/10  - ((uint16_t)GGA_Data[0])*10;                       //d 2
   GGA_Data[2] = Longitude     - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1])*10;      //d 1

 
    LongitudePart = ((GGAIData - Longitude)*60)*100000;      //degree;
   
    GGA_Data[3] = LongitudePart/1000000;                                        //m
    GGA_Data[4] = LongitudePart/100000  - GGA_Data[3]*10;                       //m
    GGA_Data[5] = 0x2E;									                              //.
    GGA_Data[6] = LongitudePart/10000   - ((uint32_t)GGA_Data[3])*100    - ((uint32_t)GGA_Data[4])*10 ;     //m
    GGA_Data[7] = LongitudePart/1000    - ((uint32_t)GGA_Data[3])*1000   - ((uint32_t)GGA_Data[4])*100    - ((uint32_t)GGA_Data[6])*10;     //m
    GGA_Data[8] = LongitudePart/100     - ((uint32_t)GGA_Data[3])*10000  - ((uint32_t)GGA_Data[4])*1000   - ((uint32_t)GGA_Data[6])*100   - ((uint32_t)GGA_Data[7])*10;     //m
    GGA_Data[9] = LongitudePart/10      - ((uint32_t)GGA_Data[3])*100000 - ((uint32_t)GGA_Data[4])*10000  - ((uint32_t)GGA_Data[6])*1000  - ((uint32_t)GGA_Data[7])*100  -  ((uint32_t)GGA_Data[8])*10;     //m
    GGA_Data[10]= LongitudePart         - ((uint32_t)GGA_Data[3])*1000000- ((uint32_t)GGA_Data[4])*100000 - ((uint32_t)GGA_Data[6])*10000 - ((uint32_t)GGA_Data[7])*1000 -  ((uint32_t)GGA_Data[8])*100 -  ((uint32_t)GGA_Data[9])*10;     //m

		 
    for (i=0;i<5;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }

    for (i=6;i<11;i++)
    {
    GGA_Data[i] = GGA_Data[i]+0x30;  
    }
   
   
}


//----------------------------------------
//void ProcessCheckToA(uint8_t CheckNum)
//----------------------------------------
void ProcessCheckToA(uint8_t CheckNum)
{									
static   uint8_t  i;
  			     
   GGA_Data[0] = CheckNum/16 ; 			       //十位
   GGA_Data[1] = CheckNum -GGA_Data[0]*16;	   //个位

   for(i=0;i<2;i++)
   {
    if(GGA_Data[i] <= 9)
	{
	 GGA_Data[i] = (GGA_Data[i] + '0') ;
	}
	else
	{
	 GGA_Data[i] = ((GGA_Data[i]-10) + 'A') ;
	}   
   }
}



//------------------------------------
//void ProcessCheck(uint8_t DNum)
//------------------------------------
uint8_t ProcessCheckResult(uint16_t UpDum,uint16_t DNum)
{
static  uint16_t i;
static  uint8_t CheckNum;

 CheckNum=0;
 for(i=UpDum+1;i<UpDum+DNum;i++)
 {
  CheckNum^=TDM_TX_Data[i]; 
 }   
 return CheckNum;
}







uint8_t ProcessATOA(uint16_t GGAIData)
{
 static uint8_t Degree;   
	
 if(abs(GGAIData)>=100)
 {
  Degree=3;
 }
 else if(abs(GGAIData)>=10)
 {
  Degree=2;
 }
 else 
 {
  Degree=1;
 }
 
 
 if(Degree==3)                                                 //132
	{
	  GGB_Data[0] = (GGAIData)/100;	                                //百位    
    GGB_Data[1] = (GGAIData)/10    - ((uint16_t)GGB_Data[0])*10;    //十位        
	  GGB_Data[2] = (GGAIData)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //个数        	

	  GGA_Data[0] = 0x30;   
	  GGA_Data[1] = GGB_Data[0] + 0x30;   
    GGA_Data[2] = GGB_Data[1] + 0x30;   
    GGA_Data[3] = GGB_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                         //13
	{
	  GGB_Data[0] = (GGAIData)/10;	                           //十位    
    GGB_Data[1] = (GGAIData)    -  GGB_Data[0]*10;   //个位        
	   	
    GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x30;   
	  GGA_Data[2] = GGB_Data[0] + 0x30;   
    GGA_Data[3] = GGB_Data[1] + 0x30;  
	}
   	else                                                                                                                  
	{
    GGB_Data[0] = (GGAIData);	                              //个位      

		GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x30; 
		GGA_Data[2] = 0x30; 
	  GGA_Data[3] = GGB_Data[0] + 0x30; 
	}

 return Degree;
}
//------------------------------------------------
//void ProcessSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------
uint8_t ProcessSFTOA(float GGAIData,uint8_t bit)
{

 static uint8_t Degree;    
 static uint8_t ZF_Flag;   
 
 if(GGAIData>=0)  
 {
  ZF_Flag=1;
 }
 else
 {
  ZF_Flag=0;
 }
//-------------------------------------
 if(abs(GGAIData)>=1000)
 {
  Degree=4;
 }
 else if(abs(GGAIData)>=100)
 {
  Degree=3;
 }
 else if(abs(GGAIData)>=10)
 {
  Degree=2;
 }
 else if(abs(GGAIData)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
   if(ZF_Flag==0)
   {
    GGAIData = -GGAIData;
   }
 //--------------------------------------------------------------  
   if(bit==2)	   				                       //"两位数字" + "." = "三位"
   {
    if(Degree==2)                                      //13.2==> 013;
	{
	  GGA_Data[0] = '0';	                               //百位    						          //.																	  	
    GGA_Data[1] = (GGAIData)/10;	                   //十位    						          //.
    GGA_Data[2] = (GGAIData)    - ((uint16_t)GGA_Data[0])*10;  //个位  		

	  GGA_Data[1] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
	}
	else if(Degree==1)                                 //9.8==>98==>9.8
	{
    GGA_Data[0] = (GGAIData*10)/10;	                               //十位
    GGA_Data[1] = 0x2E;						                       //.
    GGA_Data[2] = (GGAIData*10)    - ((uint16_t)GGA_Data[0])*10;   //个位  

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
	}
	else                                              //0.8==>08==>0.8
	{
	  GGA_Data[0] = (GGAIData*10)/10;	                               //十位
    GGA_Data[1] = 0x2E;						                       //.
    GGA_Data[2] = (GGAIData*10)    - ((uint16_t)GGA_Data[0])*10;   //个位  

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   	
	}
   }												  
//--------------------------------------------------------------
   else	 if(bit==3)									              //"三位数字" + "." = "四位"
   {
    if(Degree==3)                                                 //132==>132==>132
	{
	  GGA_Data[0] = (GGAIData)/100;	                                //百位    
    GGA_Data[1] = (GGAIData)/10    - ((uint16_t)GGA_Data[0])*10;    //十位        
	  GGA_Data[2] = (GGAIData)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //个数        	
	  GGA_Data[3] = 0x2E;						                          //.

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                            //13.2==>132==>13.2
	{
	  GGA_Data[0] = (GGAIData*10)/100;	                              //十位    
    GGA_Data[1] = (GGAIData*10)/10    - ((uint16_t)GGA_Data[0])*10;   //个位        
	  GGA_Data[2] = 0x2E;						                          //.
	  GGA_Data[3] = (GGAIData*10)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //小数        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	}
   	else                                                              //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGA_Data[0] = (GGAIData*100)/100;	                              //个位
    GGA_Data[1] = 0x2E;						                          //.
    GGA_Data[2] = (GGAIData*100)/10    - ((uint16_t)GGA_Data[0])*10;  //小位        
	  GGA_Data[3] = (GGAIData*100)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2])*10 ;  //小位        

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	}						
  }
//--------------------------------------------------------------
   else	if(bit==4)									                  //"四位数字" + "." = "五位"
   {
    if(Degree==4)                                                     //1321==>1321==>1321.
  	{
	  GGA_Data[0] = (GGAIData)/1000;	                                  //千位    
    GGA_Data[1] = (GGAIData)/100  - ((uint16_t)GGA_Data[0])*10;       //百位        
	  GGA_Data[2] = (GGAIData)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //十数        	
	  GGA_Data[3] = (GGAIData)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[2]*10) ;  //十数        	
	  GGA_Data[4] = 0x2E;						                          //.

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;       
	  GGA_Data[3] = GGA_Data[3] + 0x30;       
	  }	 	
   	else if(Degree==3)                                                       //132.1==>1321==>132.1
	  {
    GGA_Data[0] = (GGAIData*10)/1000;	                                 //千位    
    GGA_Data[1] = (GGAIData*10)/100  - ((uint16_t)GGA_Data[0])*10;       //百位        
	  GGA_Data[2] = (GGAIData*10)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //十数        	
	  GGA_Data[3] = 0x2E;						                             //.
	  GGA_Data[4] = (GGAIData*10)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[2]*10) ;  //??        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;       
	  GGA_Data[4] = GGA_Data[4] + 0x30;        
	  }	 
   	else if(Degree==2)                                                   //13.21==>1321==>13.21
	 {
    GGA_Data[0] = (GGAIData*100)/1000;	                                 //千位    
    GGA_Data[1] = (GGAIData*100)/100  - ((uint16_t)GGA_Data[0])*10;      //百位        
	  GGA_Data[2] = 0x2E;						                             //.
	  GGA_Data[3] = (GGAIData*100)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //十数        	
	  GGA_Data[4] = (GGAIData*100)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[3]*10) ;  //十数        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	  GGA_Data[4] = GGA_Data[4] + 0x30;        
	 }	 
	 else if(Degree==1)				  							        //1.321==>1321==>1.321    //0.132==>132==>0.132
	 {																											
    GGA_Data[0] = (GGAIData*1000)/1000;	                                 //千位    
	  GGA_Data[1] = 0x2E;													 //.
    GGA_Data[2] = (GGAIData*1000)/100  - ((uint16_t)GGA_Data[0])*10;     //百位        				                             //.
	  GGA_Data[3] = (GGAIData*1000)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2]*10) ;  //十数        	
	  GGA_Data[4] = (GGAIData*1000)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[2]*100)- ((uint16_t)GGA_Data[3]*10) ;  //十数        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	  GGA_Data[4] = GGA_Data[4] + 0x30;        
	 }
	 else		               
	 {											 	                     //0.321==>321==>0.321    //0.321==>321==>0.321
    GGA_Data[0] = (GGAIData*1000)/1000;	                                 //千位    
	  GGA_Data[1] = 0x2E;													 //.
    GGA_Data[2] = (GGAIData*1000)/100  - ((uint16_t)GGA_Data[0])*10;     //百位        				                             //.
	  GGA_Data[3] = (GGAIData*1000)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2]*10) ;  //十数        	
	  GGA_Data[4] = (GGAIData*1000)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[2]*100)- ((uint16_t)GGA_Data[3]*10) ;  //十数        	

	  GGA_Data[0] = 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	  GGA_Data[4] = GGA_Data[4] + 0x30;        		
	  }										   
   }
   
   return ZF_Flag;										
}
//------------------------------------------------
//void ProcessITOA(uint8_t GGAIData,uint8_t bit)
//------------------------------------------------
void ProcessITOA(uint8_t GGAIData)
{									
 static  uint8_t  i;
  			     
   GGA_Data[0] = GGAIData/10 ;				   //十位
   GGA_Data[1] = GGAIData -GGA_Data[0]*10;	   //个位

   for(i=0;i<2;i++)
   {
    GGA_Data[i] = GGA_Data[i] + 0x30;   
   }
}


//------------------------------------------------
//void ProcessITOA(uint8_t GGAIData,uint8_t bit)
//------------------------------------------------
void ProcessIITOA(uint16_t GGAIData)
{									
 static  uint8_t  i;
  			     
   GGA_Data[0] = GGAIData/1000 ;				   //十位
   GGA_Data[1] = GGAIData/100 - GGA_Data[0]*10;	   //个位
   GGA_Data[2] = GGAIData/10  - GGA_Data[0]*100  - GGA_Data[1]*10;	   //个位
	 GGA_Data[3] = GGAIData     - GGA_Data[0]*1000 - GGA_Data[1]*100 - GGA_Data[2]*10;	   //个位
	 
   for(i=0;i<4;i++) 
   {
    GGA_Data[i] = GGA_Data[i] + 0x30;   
   }
}




uint8_t ProcessAgeTOA(uint16_t GGAIData)
{
 static uint8_t Degree;   
	
 if(abs(GGAIData)>=100)
 {
  Degree=3;
 }
 else if(abs(GGAIData)>=10)
 {
  Degree=2;
 }
 else 
 {
  Degree=1;
 }
 
 
 if(Degree==3)                                                 //132
	{
	  GGB_Data[0] = (GGAIData)/100;	                                //百位    
    GGB_Data[1] = (GGAIData)/10    - ((uint16_t)GGB_Data[0])*10;    //十位        
	  GGB_Data[2] = (GGAIData)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //个数        	
	  
	  GGA_Data[0] = GGB_Data[0] + 0x30;   
    GGA_Data[1] = GGB_Data[1] + 0x30;   
		GGA_Data[2] = 0x2E;	 
    GGA_Data[3] = GGB_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                         //13
	{
	  GGB_Data[0] = (GGAIData)/10;	                           //十位    
    GGB_Data[1] = (GGAIData)    -  GGB_Data[0]*10;   //个位        
	
		GGA_Data[0] = GGB_Data[0] + 0x30;   
		GGA_Data[1] = 0x2E;	 
    GGA_Data[2] = GGB_Data[1] + 0x30; 
    GGA_Data[3] = 0x30;   
	}
  else                                                                                                                  
	{     
		GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x2E;	 
		GGA_Data[2] = 0x30; 
	  GGA_Data[3] = 0x30; 
	}

 return Degree;
}



//-------------------------------
//void  GetGPGGA(void)
//-------------------------------
void  GetLGPGGA(void)
{
uint8_t i=0;
uint8_t ZF_Flag=0;
uint8_t  CheckNumData;
	
	
uint8_t  Hea_Data[6];         //$GPGGA    = 6              ??
uint8_t  TEN_Data[1];         //*         = 1               *

	
//-----------------GGA------------------------------------------
uint8_t  UTC_Data[9];          //hhmmss.ss  =  9            UTC??
uint8_t  Lat_Data[15];         //ddmm.mmmmmmm = 12          ??
uint8_t  SNth_Data[1];		     //N S
uint8_t  Lon_Data[15];         //dddmm.mmmmmmm= 13          ??
uint8_t  WEst_Data[1];         //W E
uint8_t  GPSQ_Data[1];         //1,2,3,4                    GPSQuality
uint8_t  NumS_Data[2];         //08                         NumOfSatsInUse
uint8_t  HDOP_Data[4];         //1321/0132/13.2/1.32/0.13/  HDOP	    ?????? ???4?;
uint8_t  Alti_Data[8];         //1020/102./10.2/0.10/0.01/  Altitude	?????? ???4?;
uint8_t  Unit_Data[1];         //M                          Unites;
uint8_t  Geoi_Data[5];         //980/098/9.8/0.9/           Geoidal		?????? ???3?;
uint8_t  DTim_Data[4];         //                           DTime 		?????? ???1?;
uint8_t  DFID_Data[4];         //0000                       DGPS ID					 ???4?;
	
//-------------------------------
//          GPGGA
//-------------------------------

//--------字段----数-------------

//----------0-----6---------------
//帧头
//--------------------------------

 	if(GNGA_DGet_Flag)
  {
   Hea_Data[0] = '$';
   Hea_Data[1] = 'G';
   Hea_Data[2] = 'N';
   Hea_Data[3] = 'G';
   Hea_Data[4] = 'G';
   Hea_Data[5]=  'A';
 }
 else
 {
   Hea_Data[0] = '$';
   Hea_Data[1] = 'G';
   Hea_Data[2] = 'P';
   Hea_Data[3] = 'G';
   Hea_Data[4] = 'G';
   Hea_Data[5]=  'A';
 }


 
//----------1-----9----------------
//UTC时间
//--------------------------------

  //ProcessITOA(GINavResult.UtcTime.Hour);
  ProcessITOA(GPGGAData.Hour);
  UTC_Data[0]  = GGA_Data[0];    //0
  UTC_Data[1]  = GGA_Data[1];    //1

  //ProcessITOA(GINavResult.UtcTime.Minute);
  ProcessITOA(GPGGAData.Minute);
  UTC_Data[2]  = GGA_Data[0];	//2
  UTC_Data[3] = GGA_Data[1];	//3
 
  //ProcessITOA(GINavResult.UtcTime.Second); 
  ProcessITOA(GPGGAData.Second);

  UTC_Data[4] = GGA_Data[0];	//4
  UTC_Data[5] = GGA_Data[1];	//5

  UTC_Data[6] = '.';	        //6

  //ProcessITOA(GINavResult.UtcTime.MillSecond/10);
	if(GPGGAData.MSecond>=100)
	{
	  GPGGAData.MSecond=GPGGAData.MSecond/10;
	}
	
  ProcessITOA(GPGGAData.MSecond);
  UTC_Data[7] = GGA_Data[0];	//7
  UTC_Data[8] = GGA_Data[1];	//8

 

//----------2------10----------------
//维度
//--------------------------------
 //----------2------10----------------
//维度
//--------------------------------
 if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
 {
    ProcessLatToA(GPGGAData.Latitude);   //10-19;
 }
 else
 { 
    ProcessLatToA(GINavResult.Position.Lat*RAD2DEG);     //10-19;	 		
 }

  for(i=0;i<10;i++)
  {
   Lat_Data[i] = GGA_Data[i];  
  }
	
//----------3------1----------------	
//南北
//--------------------------------
	if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
	{		
		if((GPGGAData.SNth=='N')||(GPGGAData.SNth=='S'))
		{
		 SNth_Data[0] = GPGGAData.SNth;	         //21; < 0 = South, > 0 = North
		}
		else
		{				
			if(NorthSouthSelect==1)
			{
			 SNth_Data[0] ='N';
			}
			else 	if(NorthSouthSelect==2)
			{
			 SNth_Data[0] ='S';
			}
			else
			{
			 SNth_Data[0] ='0';
			}
		}	
	}
	else
	{
		if((GPGGAData.SNth=='N')||(GPGGAData.SNth=='S'))
		{
		 SNth_Data[0] = GPGGAData.SNth;	         //21; < 0 = South, > 0 = North
		}
		else
		{
			if(NorthSouthSelect==1)
			{
			 SNth_Data[0] ='N';
			}
			else 	if(NorthSouthSelect==2)
			{
			 SNth_Data[0] ='S';
			}
			else
			{
			 SNth_Data[0] ='0';
			}
		}   
  }
 
 
 if(GPGGAData.SNth=='N')
 {
  NorthSouthSelect=1;
 }
 else if(GPGGAData.SNth=='S')
 {
  NorthSouthSelect=2; 
 } 

 
 //--------------------------------
 //如果车库启动,且没有见过GPS
 //--------------------------------
 	if((GPGGAData.SNth=='N')||(GPGGAData.SNth=='S'))
	{
		 SNth_Data[0] = GPGGAData.SNth;	         //21; < 0 = South, > 0 = North
		
	 	 g_GINavInfo.GetGpsDataFlag=1;
	}
	else
	{
   if((RSE_SGet_Flag==1)&&(g_GINavInfo.GetGpsDataFlag==0)) //没有得到Gps，且车库启动
   {
		if(NorthSouthBack==2)
		{
			 SNth_Data[0] ='S';
		}
		else
		{
			 SNth_Data[0] ='N';
		} 
   }
  }
//----------4-------11----------------	
//经度
//-------------------------------- 
 	if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
 {
   ProcessLongToA(GPGGAData.Longitude);   //10-19;	
 }
 else
 {
	 ProcessLongToA(GINavResult.Position.Lon*RAD2DEG);     //10-19;	 			
 } 
 
  for(i=0;i<11;i++)
  {
   Lon_Data[i] = GGA_Data[i];  
  }
	
	

  
//----------5-------1----------------	
//东西
//--------------------------------

  
  if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
	{
	 if((GPGGAData.WEst=='W')||(GPGGAData.WEst=='E'))
	 {
    WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East
   }
	 else
	 {
	   if(WestEastSelect==1)
			{
			 WEst_Data[0] ='W';
			}
			else 	if(WestEastSelect==2)
			{
			 WEst_Data[0] ='E';
			}
			else
			{
			 WEst_Data[0] ='0';
			}
	 }
	}
	else
	{
		if((GPGGAData.WEst=='W')||(GPGGAData.WEst=='E'))
		{
		 WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East
		}
		else
		{
		 if(WestEastSelect==1)
			{
			 WEst_Data[0] ='E';
			}
			else if(WestEastSelect==2)
			{
			 WEst_Data[0] ='W';
			}
		  else
			{
			 WEst_Data[0] ='0';
			}
		}  
  }
	
 
  
 if(GPGGAData.WEst=='E')
 {
  WestEastSelect=1;
 }
 else if(GPGGAData.SNth=='W')
 {
  WestEastSelect=2; 
 } 
 

 //--------------------------------
 //如果车库启动,且没有见过GPS
 //--------------------------------
 	if((GPGGAData.WEst=='W')||(GPGGAData.WEst=='E'))
	{
		 WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East
		
		 g_GINavInfo.GetGpsDataFlag=1;
	}
	else
	{
   if((RSE_SGet_Flag==1)&&(g_GINavInfo.GetGpsDataFlag==0))
   {
	  if(WestEastBack==2)
		{
		 WEst_Data[0] ='W';
		}
		else 
		{
		 WEst_Data[0] ='E';
		} 
   }
  }

//----------6-------6-----------------
//标志
//--------------------------------
  /*if(GPGGAData.GPSQuality==0x04) 
	{
   GPSQ_Data[0] = GPGGAData.GPSQuality+0x30;  //37;    
	}
	 else
  {
   GPSQ_Data[0] = 0x36;                       //37;    
  }
 */

	if((GetAgeDataFlag)&&(GPGGAData.NumOfSatsInUse>0)&&(SGPS_DGet_Flag))
	{			
		if(GINavResult.GpsHighFlag==1)
		{
		 if(GPGGAData.DTime<59)
		 {
			GPSQ_Data[0] = 0x32;  //37;				  	 
		 }
		 else
		 {
			GPSQ_Data[0] = GPGGAData.GPSQuality+0x30;  //37;				  	 
		 }
	 }
	 else
	 {			
		if(GetAgeData<590)
		{
		 GPSQ_Data[0] = 0x32;  //37;		
		} 
		else
		{
		 GPSQ_Data[0] = GPGGAData.GPSQuality+0x30;  //37;	
		}			
	 }		  
	}	
	else
	{
	 GPSQ_Data[0] = GPGGAData.GPSQuality+0x30;  //37;
	}

		
	if(IS_INS_ALIGNED(g_GINavInfo.INSState))
  {
	  if(GPGGAData.GPSQuality==0)
	  {
     GPSQ_Data[0] =0x36;  //37;		  
	  }
	}  
//-----------7------2----------------- 
//星数
//--------------------------------

	if((GetAgeDataFlag==0)||(SGPS_DGet_Flag==0))
	{
	 ProcessITOA(GPGGAData.NumOfSatsInUse);	 
	 NumS_Data[0]  = GGA_Data[0];	          //39;
	 NumS_Data[1]  = GGA_Data[1];            //40;  
	}
	else
	{   
		if(GINavResult.GpsHighFlag==1)
		{
			if(GPGGAData.DTime<59)
			{
			 if(GPGGAData.NumOfSatsInUse>8)
		   {
			  GPGGAData.NumOfSatsInUse=8;
		   }
			}			
		}
		else
		{
			if(GetAgeData<590)
			{
		   if(GPGGAData.NumOfSatsInUse>8)
		   {
			 GPGGAData.NumOfSatsInUse=8;
		   }
	   }
		}
		
	 ProcessITOA(GPGGAData.NumOfSatsInUse);	 
	 NumS_Data[0]  = GGA_Data[0];	          //39;
	 NumS_Data[1]  = GGA_Data[1];            //40; 
	} 
  
//-----------8------4----------------
//精度
//--------------------------------
		
	ProcessSFTOA(GINavResult.GstDeta,3); 	
  HDOP_Data[0] = GGA_Data[0];	           //42;
  HDOP_Data[1] = GGA_Data[1];              //43; 
  HDOP_Data[2] = GGA_Data[2];              //44; 
  HDOP_Data[3] = GGA_Data[3];              //45; 
  
//-----------9------4-----------	
//高度
//--------------------------------
	if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
 {
  ZF_Flag=ProcessSFTOA(GPGGAData.Altitude,4);  	 
 }
 else
 {	
  ZF_Flag=ProcessSFTOA(GINavResult.Position.Alt,4);   
}  
 
	 

	if(ZF_Flag==1)
  {
   Alti_Data[0] = GGA_Data[0];	            //47;
   Alti_Data[1] = GGA_Data[1];              //48; 
   Alti_Data[2] = GGA_Data[2];              //49; 
   Alti_Data[3] = GGA_Data[3];              //50; 
   Alti_Data[4] = GGA_Data[4];              //50;   
  }
  else
  {
   Alti_Data[0] = '-';	                    //47;
   Alti_Data[1] = GGA_Data[0];              //48; 
   Alti_Data[2] = GGA_Data[1];              //49; 
   Alti_Data[3] = GGA_Data[2];              //50; 
   Alti_Data[4] = GGA_Data[3];              //50;     
  }
	

 
//-----------10----1------------	
//单位
//--------------------------------
 if((Ttx_data[0]==Ddx_data[0])&&(Ttx_data[1]==Ddx_data[1]))
 {
  Unit_Data[0] = 'M';	                  //52;  
 }
 else
 {
  Unit_Data[0] = 'm';	                  //52;  
 }
 

//-----------11----3------------
//水平高度
//--------------------------------
 if(Debug_Flag)
 {
 ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Pitch*RAD2DEG,4);
 }
 else
 {
 ZF_Flag=ProcessSFTOA(GPGGAData.Geoidal,4);
 } 
 
  if(ZF_Flag)
  {
  Geoi_Data[0] = GGA_Data[0];	           //54;
  Geoi_Data[1] = GGA_Data[1];              //55; 
  Geoi_Data[2] = GGA_Data[2];              //56; 
  Geoi_Data[3] = GGA_Data[3];              //56; 
  Geoi_Data[4] = GGA_Data[4];              //56;  
  }
  else
  {
  Geoi_Data[0] = '-';	           //54;
  Geoi_Data[1] = GGA_Data[0];              //55; 
  Geoi_Data[2] = GGA_Data[1];              //56; 
  Geoi_Data[3] = GGA_Data[2];              //56; 
  Geoi_Data[4] = GGA_Data[3];              //56;   
  }
 
//------------12---1-------------	
//单位
//--------------------------------


//------------13---1-------------	
//DPS Time		GGA处理
//--------------------------------

	if((GINavResult.GpsHighFlag==1)||(GetAgeDataFlag==0)||(SGPS_DGet_Flag==0))  //
	{	
		ZF_Flag = ProcessSFTOA(GPGGAData.DTime,3);
	}
	else
	{
		ZF_Flag = ProcessAgeTOA(GetAgeData);
	}


 DTim_Data[0] = GGA_Data[0];	           //42;
 DTim_Data[1] = GGA_Data[1];             //43; 
 DTim_Data[2] = GGA_Data[2];             //44; 
 DTim_Data[3] = GGA_Data[3];             //45;  		    

//------------14---4-------------	
//ID 差分系统，GGA处理
//--------------------------------
 /*if(Debug_Flag)
 {
	if(GINavResult.GpsHighFlag==1)
	{
   DFID_Data[0]=0x30;
   DFID_Data[1]=0x30;
   DFID_Data[2]=0x30;
   DFID_Data[3]=0x31;		
  }
  else
	{
   DFID_Data[0]=0x30;
   DFID_Data[1]=0x30;
   DFID_Data[2]=0x30;
   DFID_Data[3]=0x30;		
  }  
 }
 */
 
    ProcessIITOA(GINavResult.GoodCount);  

   DFID_Data[0]=GGA_Data[0];
   DFID_Data[1]=GGA_Data[1];
   DFID_Data[2]=GGA_Data[2];
   DFID_Data[3]=GGA_Data[3];		
	 
//------------15---1-------------	
//结尾
//--------------------------------
  TEN_Data[0]  = '*';                	  //64; 
//------------15---2-------------
//CheckNum
//--------------------------------


if(GGA_UP_Num>=100)
	GGA_UP_Num=100;

//--------------------------------------------------

	  
 //----------0--------6---------------
 //$GPGGA			 0-5,6
 //-----------------------------------

  for(i=0;i<6;i++)		          
  TDM_TX_Data[GGA_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[GGA_UP_Num+6] = ',';
 
 //----------1--------9---------------
 //UTC hhmmss.ss    7-15,16
 //-----------------------------------
  for(i=0;i<9;i++)		          
  TDM_TX_Data[GGA_UP_Num+7+i] = UTC_Data[i];

  TDM_TX_Data[GGA_UP_Num+16] = ',';

 //----------2--------10---------------
 //维度 ddmm.mmmmm    17-38,29
 //-----------------------------------
  for(i=0;i<10;i++)		          
  TDM_TX_Data[GGA_UP_Num+17+i] = Lat_Data[i];
	
  TDM_TX_Data[GGA_UP_Num+27] = ',';
 //----------3--------1---------------
 //维度 N S           30,31
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+28] = SNth_Data[0];
  TDM_TX_Data[GGA_UP_Num+29] = ',';
 

 //----------4--------11---------------
 //经度dddmm.mmmmm     32-44,45
 //-----------------------------------
  for(i=0;i<11;i++)		          
  TDM_TX_Data[GGA_UP_Num+30+i] = Lon_Data[i];

  TDM_TX_Data[GGA_UP_Num+41] = ',';


 //----------5-----------1---------------
 //东西 W E            46,47
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+42] = WEst_Data[0];
  TDM_TX_Data[GGA_UP_Num+43] = ',';

 //----------6-----------1---------------
 //GPSQuality          48,49
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+44] = GPSQ_Data[0];
  TDM_TX_Data[GGA_UP_Num+45] = ',';
 //----------7-----------2---------------
 //NumOfStar          50-51,52
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+46] = NumS_Data[0];
  TDM_TX_Data[GGA_UP_Num+47] = NumS_Data[1];
  TDM_TX_Data[GGA_UP_Num+48] = ',';
 //----------8-----------4---------------
 //HDOP               53-56,57
 //-----------------------------------
  for(i=0;i<4;i++)		          
  TDM_TX_Data[GGA_UP_Num+49+i] = HDOP_Data[i];
 
  TDM_TX_Data[GGA_UP_Num+53] = ',';

 //----------9-----------4---------------
 //Alititude          58-62,63
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GGA_UP_Num+54+i] = Alti_Data[i];
 
  TDM_TX_Data[GGA_UP_Num+59] = ',';
 //----------10-----------1------------
 //M                   64,65
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+60] = Unit_Data[0];
  TDM_TX_Data[GGA_UP_Num+61] = ',';

 //----------11-----------3---------------
 //Geoidal            66-70,71
 //--------------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GGA_UP_Num+62+i]= Geoi_Data[i];
  
  TDM_TX_Data[GGA_UP_Num+67] = ',';

 //----------12-----------1------------
 //M                   72,73
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+68] = Unit_Data[0];
  TDM_TX_Data[GGA_UP_Num+69] = ',';

 //----------12-----------1------------
 //DTime                74,75
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+70] = DTim_Data[0];
  TDM_TX_Data[GGA_UP_Num+71] = DTim_Data[1];
  TDM_TX_Data[GGA_UP_Num+72] = DTim_Data[2];
  TDM_TX_Data[GGA_UP_Num+73] = DTim_Data[3];
  TDM_TX_Data[GGA_UP_Num+74] = ',';

 //----------13-----------4---------------
 //ID                 76-79,80
 //--------------------------------------

  for(i=0;i<4;i++)		          
  TDM_TX_Data[GGA_UP_Num+75+i]= DFID_Data[i];
 //----------14-----------1---------------
 //CheckSum           80,81-82
 //-------------------------------------- 
  TDM_TX_Data[GGA_UP_Num+79] = TEN_Data[0];         
  
  CheckNumData    = ProcessCheckResult(GGA_UP_Num,79);
  ProcessCheckToA(CheckNumData);
  TDM_TX_Data[GGA_UP_Num+80] = GGA_Data[0];	
  TDM_TX_Data[GGA_UP_Num+81] = GGA_Data[1];	          		
       	  
 //----------15-----------1---------------
 //回车换行			   83 84
 //-------------------------------------- 
  TDM_TX_Data[GGA_UP_Num+82] = 0x0D;	          
  TDM_TX_Data[GGA_UP_Num+83] = 0x0A;	    
 

 //---------------------------------------
 //启动发送
 //---------------------------------------
  GGA_TX_Max     = 84;    

  ZDA_UP_Num     = GGA_UP_Num + GGA_TX_Max;
}


//----------------------------
//void  GetGPVTG(void)
//----------------------------
void  GetGPVTG(void)
{
static float GroundSpeed=0;
static uint8_t   i=0;

uint8_t  Hea_Data[6];         //$GPGGA    = 6              ??
uint8_t  CheckNumData;

uint8_t  VtgYaw_Data[10];
uint8_t  VtgNotSpd_Data[7];
uint8_t  VtgKmSpd_Data[7];
uint8_t  VtgYawNum;

uint8_t  NotSpdNum;
uint8_t  KmSpdNum;


//--------??----?-------------

	
 if(GPVTGData.Gp_PN_Kind==1)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'P';
  Hea_Data[3] = 'V';
  Hea_Data[4] = 'T';
  Hea_Data[5] = 'G';
 }
 else if(GPVTGData.Gp_PN_Kind==2)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'L';
  Hea_Data[3] = 'V';
  Hea_Data[4] = 'T';
  Hea_Data[5] = 'G';
 }
 else if(GPVTGData.Gp_PN_Kind==3)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'B';
  Hea_Data[2] = 'D';
  Hea_Data[3] = 'V';
  Hea_Data[4] = 'T';
  Hea_Data[5] = 'G';
 }
 else
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'N';
  Hea_Data[3] = 'V';
  Hea_Data[4] = 'T';
  Hea_Data[5] = 'G';  
 }

//----------2-----9------------------
// 方向
//---------------------------------
 
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
  {
   VtgYawNum = ProcessZLSFTOA(GPVTGData.Ttrack,3);     				
  }
  else
  {
	 if(GPGGAData.GPSQuality==4)
	 {
    VtgYawNum = ProcessZLSFTOA(GPVTGData.Ttrack,3); 	
   }
   else
   {	
    VtgYawNum = ProcessZLSFTOA(GINavResult.Attitude.Heading*RAD2DEG,3);    
   }    
	} 
   for(i=0;i<VtgYawNum;i++)
   {
    VtgYaw_Data[i]= GGA_Data[i];  
   }	
	 
//----------7-----5------------------   
// 海里地速
//---------------------------------
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
 {
   NotSpdNum = ProcessZLSFTOA(GPVTGData.speedNot,3);
 }
 else
 {
	 if(GPGGAData.GPSQuality==4)
	 {
	  NotSpdNum = ProcessZLSFTOA(GPVTGData.speedNot,3);
	 }
	 else
	 {
	  GroundSpeed=sqrt(GINavResult.Velocity.Ve*GINavResult.Velocity.Ve+GINavResult.Velocity.Vn*GINavResult.Velocity.Vn)/0.5144444444444;
	  NotSpdNum = ProcessZLSFTOA(GroundSpeed,3); 
	 }
 }	
 
  for(i=0;i<NotSpdNum;i++)
  {
    VtgNotSpd_Data[i]= GGA_Data[i];  
  }
 
 //-------------------------------------
 //         公里速度
 //-------------------------------------
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
  {
   KmSpdNum = ProcessZLSFTOA(GPVTGData.speedkm,3);
  }
  else
  {
	 if(GPGGAData.GPSQuality==4)
	 {		
	  KmSpdNum = ProcessZLSFTOA(GPVTGData.speedkm,3);
	 }
   else
	 {		
    GroundSpeed=sqrt(GINavResult.Velocity.Ve*GINavResult.Velocity.Ve+GINavResult.Velocity.Vn*GINavResult.Velocity.Vn)*60*60/1000.0;	 //每小时海里....
    KmSpdNum = ProcessZLSFTOA(GroundSpeed,3);  
	 }
  }	
 
  for(i=0;i<KmSpdNum;i++)
  {
    VtgKmSpd_Data[i]= GGA_Data[i];  
  }




//----------------------------
//            帧头
//----------------------------
 for(i=0;i<6;i++)
 {
  TDM_TX_Data[i] = Hea_Data[i];
 }

 TDM_TX_Data[6] =  ',';
//----------------------------
//         运动方向  0.012;
//----------------------------
 for(i=0;i<VtgYawNum;i++)
 {
 TDM_TX_Data[7+i] = VtgYaw_Data[i];
 }

 TDM_TX_Data[7+VtgYawNum]=',';

//----------------------------
//         运动方向标志
//----------------------------
 TDM_TX_Data[8+VtgYawNum]='T';
 TDM_TX_Data[9+VtgYawNum]=',';

//----------------------------
//          磁场方向
//----------------------------
 for(i=0;i<VtgYawNum;i++)
 {
 TDM_TX_Data[10+VtgYawNum+i] = VtgYaw_Data[i];
 }							  
 TDM_TX_Data[10+2*VtgYawNum]=',';

//----------------------------
//          磁场方向标志
//----------------------------
 TDM_TX_Data[11+2*VtgYawNum]='M';
 TDM_TX_Data[12+2*VtgYawNum]=','; 

//----------------------------
//          海里速度
//----------------------------
 for(i=0;i<NotSpdNum;i++)
 {
 TDM_TX_Data[13+2*VtgYawNum+i] = VtgNotSpd_Data[i];
 }							  
 TDM_TX_Data[13+2*VtgYawNum+NotSpdNum]=',';

 //----------------------------
//          海里方向标志
//----------------------------
 TDM_TX_Data[14+2*VtgYawNum+NotSpdNum]='N';
 TDM_TX_Data[15+2*VtgYawNum+NotSpdNum]=',';

//--------------------------------
//            公里速度
//--------------------------------
 for(i=0;i<KmSpdNum;i++)
 {
 TDM_TX_Data[16+2*VtgYawNum+NotSpdNum+i] = VtgKmSpd_Data[i];
 }							  
 TDM_TX_Data[16+2*VtgYawNum+NotSpdNum+KmSpdNum]=',';

//----------------------------
//          海里方向标志
//----------------------------
 TDM_TX_Data[17+2*VtgYawNum+NotSpdNum+KmSpdNum]='K';
 TDM_TX_Data[18+2*VtgYawNum+NotSpdNum+KmSpdNum]=',';

//-----------------------------------
//          定位模式标志
//-----------------------------------
 TDM_TX_Data[19+2*VtgYawNum+NotSpdNum+KmSpdNum]=GPVTGData.PositMode;
 TDM_TX_Data[20+2*VtgYawNum+NotSpdNum+KmSpdNum]='*';

 CheckNumData    = ProcessCheckResult(0,20+2*VtgYawNum+NotSpdNum+KmSpdNum);
 ProcessCheckToA(CheckNumData);

 TDM_TX_Data[21+2*VtgYawNum+NotSpdNum+KmSpdNum] = GGA_Data[0];	
 TDM_TX_Data[22+2*VtgYawNum+NotSpdNum+KmSpdNum] = GGA_Data[1];	          		
         

 TDM_TX_Data[23+2*VtgYawNum+NotSpdNum+KmSpdNum]  = 0x0D;
 TDM_TX_Data[24+2*VtgYawNum+NotSpdNum+KmSpdNum]  = 0x0A;


 VTG_TX_Max = 25+2*VtgYawNum+NotSpdNum+KmSpdNum;   
 GGA_UP_Num = VTG_TX_Max;
 ZDA_UP_Num = VTG_TX_Max;


}

//----------------------
//void  GetGPRMC(void)
//----------------------
void  GetGPRMC(void)
{
float GroundSpeed;
float MisAngle;
	
uint8_t  CheckNumData;	
	

uint8_t i;
uint8_t ZF_Flag;	
uint8_t  Hea_Data[6];         //$GPGGA    = 6              ??
uint8_t  TEN_Data[1];
	
//----------------RMC-------------------------------------------------
uint8_t  UTC_Data[9];          //hhmmss.ss  =  9            UTC??
uint8_t  Lat_Data[15];         //ddmm.mmmmmmm = 12          ??
uint8_t  SNth_Data[1];		     //N S
uint8_t  Lon_Data[15];         //dddmm.mmmmmmm= 13          ??
uint8_t  WEst_Data[1];         //W E

uint8_t  Vad_Data[1];          //A V		 
uint8_t  Spd_Data[5];          //138.1/13.81/1.381/0.1831   //Speed Over Ground, Knots  1.852??/??;??????100knots ?????? ???4?;
uint8_t  Tra_Data[9];          //128.1/12.81/1.281/0.1281   //Track made good; Degree						 
uint8_t  Dat_Data[6];          //ddmmyy                     //??
uint8_t  Mag_Data[4];          //980/098/9.8/0.9            //Magnetic Variatio     ??????	???3?
uint8_t  Mod_Data[1];          //A,D,E,N                    //Mode Indicator        Ublox
uint8_t  BAngle_Data[5];

//-------------------------------
//          GPRMC
//-------------------------------

//--------字段----数-------------

 if(GPRMCData.Gp_PN_Kind==1)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'P';
  Hea_Data[3] = 'R';
  Hea_Data[4] = 'M';
  Hea_Data[5] = 'C';
 }
 else if(GPRMCData.Gp_PN_Kind==2)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'L';
  Hea_Data[3] = 'R';
  Hea_Data[4] = 'M';
  Hea_Data[5] = 'C';  
 }
 else if(GPRMCData.Gp_PN_Kind==3)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'B';
  Hea_Data[2] = 'D';
  Hea_Data[3] = 'R';
  Hea_Data[4] = 'M';
  Hea_Data[5] = 'C';  
 }
 else 
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'N';
  Hea_Data[3] = 'R';
  Hea_Data[4] = 'M';
  Hea_Data[5] = 'C';  
 }





 //----------1-----9----------------
//UTC时间
//--------------------------------

  //ProcessITOA(GINavResult.UtcTime.Hour);
  ProcessITOA(GPRMCData.Hour);
  UTC_Data[0]  = GGA_Data[0];    //0
  UTC_Data[1]  = GGA_Data[1];    //1

  //ProcessITOA(GINavResult.UtcTime.Minute);
  ProcessITOA(GPRMCData.Minute);
  UTC_Data[2]  = GGA_Data[0];	//2
  UTC_Data[3] = GGA_Data[1];	//3
 
  
  //ProcessITOA(GINavResult.UtcTime.Second);
  ProcessITOA(GPRMCData.Second);
  UTC_Data[4] = GGA_Data[0];	//4
  UTC_Data[5] = GGA_Data[1];	//5

  UTC_Data[6] = '.';	        //6

  //ProcessITOA(GINavResult.UtcTime.MillSecond/10);
	if(GPRMCData.MSecond>=100)
	{
	 GPRMCData.MSecond=GPRMCData.MSecond/10;
	}
	
  ProcessITOA(GPRMCData.MSecond);

  UTC_Data[7] = GGA_Data[0];	//7
  UTC_Data[8] = GGA_Data[1];	//8


//----------2------10----------------
//维度
//--------------------------------	

 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
 {
	 Vad_Data[0] = GPRMCData.DataValid; 
 }
 else
 {
   Vad_Data[0] = 'A'; 
 }
//----------2------10----------------
//维度
//--------------------------------
//----------2------10----------------
//维度
//--------------------------------
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
 {  
    ProcessLatToA(GPRMCData.Latitude);   //10-19;		
 }
 else
 {		
	  ProcessLatToA(GINavResult.Position.Lat*RAD2DEG);   //10-19;	  	
 }
 
 
  for(i=0;i<10;i++)
  {
   Lat_Data[i] = GGA_Data[i];  
  }	
  
  										 
	
	//----------3------1----------------	
//南北
//--------------------------------
	if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
	{		
		if((GPRMCData.SNth=='N')||(GPRMCData.SNth=='S'))
		{
		 SNth_Data[0] = GPGGAData.SNth;	         //21; < 0 = South, > 0 = North
		}
		else
		{				
			 if(NorthSouthSelect==1)
			{
			 SNth_Data[0] ='N';
			}
			else 	if(NorthSouthSelect==2)
			{
			 SNth_Data[0] ='S';
			}
			else
			{
			 SNth_Data[0] ='0';
			}
		}	
	}
	else
	{
		if((GPRMCData.SNth=='N')||(GPRMCData.SNth=='S'))
		{
		 SNth_Data[0] = GPGGAData.SNth;	         //21; < 0 = South, > 0 = North
		}
		else
		{
			 if(NorthSouthSelect==1)
			{
			 SNth_Data[0] ='N';
			}
			else 	if(NorthSouthSelect==2)
			{
			 SNth_Data[0] ='S';
			}
			else
			{
			 SNth_Data[0] ='0';
			}
		}   
  }

	
 //--------------------------------
 //如果车库启动
 //--------------------------------
	if((GPRMCData.SNth=='N')||(GPRMCData.SNth=='S'))
	{
		 SNth_Data[0] = GPRMCData.SNth;	         //21; < 0 = South, > 0 = North
		 g_GINavInfo.GetGpsDataFlag=1;
	}
	else
	{
   if((RSE_SGet_Flag==1)&&(g_GINavInfo.GetGpsDataFlag==0))
   { 
		 if(NorthSouthBack==2)
		 {
		 SNth_Data[0] ='S';
		 }
		 else
		 {
		  SNth_Data[0] ='N';
		 } 
    }
   }
	
//----------4-------11----------------	
//经度
//--------------------------------
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
 {
    ProcessLongToA(GPRMCData.Longitude);   //10-19	 
 }
 else
 { 
    ProcessLongToA(GINavResult.Position.Lon*RAD2DEG);   //10-19;	 	 	 
 }
 
 
  for(i=0;i<11;i++)
  {
   Lon_Data[i] = GGA_Data[i];  
  }											 
  
	

  
	
//----------5-------1----------------	
//东西
//--------------------------------

  
//----------5-------1----------------	
//东西
//--------------------------------

  
  if((!IS_INS_ALIGNED(g_GINavInfo.INSState)))
	{
	 if((GPGGAData.WEst=='W')||(GPGGAData.WEst=='E'))
	 {
    WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East
   }
	 else
	 {
	   if(WestEastSelect==1)
			{
			 WEst_Data[0] ='E';
			}
			else 	if(WestEastSelect==2)
			{
			 WEst_Data[0] ='W';
			}
			else
			{
			 WEst_Data[0] ='0';
			}
	 }
	}
	else
	{
		if((GPGGAData.WEst=='W')||(GPGGAData.WEst=='E'))
		{
		 WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East
		}
		else
		{
		   if(WestEastSelect==1)
			{
			 WEst_Data[0] ='E';
			}
			else 	if(WestEastSelect==2)
			{
			 WEst_Data[0] ='W';
			}
			else
			{
			 WEst_Data[0] ='0';
			}
		}  
  }	
	
	
 //----------------------------------------
 //
 //---------------------------------------- 
	
 if(GPGGAData.WEst=='E')
 {
  WestEastSelect=1;
 }
 else if(GPGGAData.SNth=='W')
 {
  WestEastSelect=2; 
 } 
 
 
 //--------------------------------
 //如果车库启动
 //--------------------------------
 	if((GPRMCData.SNth=='N')||(GPRMCData.SNth=='S'))
	{
		 SNth_Data[0] = GPRMCData.SNth;	         //21; < 0 = South, > 0 = North
		 g_GINavInfo.GetGpsDataFlag=1;
	}
	else
  {
   if((RSE_SGet_Flag==1)&&(g_GINavInfo.GetGpsDataFlag==0))
   {
	  if(WestEastBack==2)
		{
		 WEst_Data[0] ='W';
		}
		else 
		{
		 WEst_Data[0] ='E';
		} 
   }
  }
  
//----------7-----5------------------   
// 地速
//---------------------------------
 if(!IS_INS_ALIGNED(g_GINavInfo.INSState)) 
 {
   GroundSpeed=GPRMCData.GroundSpeed;
 }
 else
 {
	 GroundSpeed=sqrt(GINavResult.Velocity.Ve*GINavResult.Velocity.Ve+GINavResult.Velocity.Vn*GINavResult.Velocity.Vn)/0.5144444444444;	 
 }	
 
  ProcessSFTOA(GroundSpeed,4);  
 
  Spd_Data[0]= GGA_Data[0];
  Spd_Data[1]= GGA_Data[1];
  Spd_Data[2]= GGA_Data[2];
  Spd_Data[3]= GGA_Data[3];
  Spd_Data[4]= GGA_Data[4];

//----------8-----5------------------
// 方向
//---------------------------------
  if(!IS_INS_ALIGNED(g_GINavInfo.INSState)) 
	{
	 ZF_Flag=ProcessSFTOA(GPRMCData.Course,4);  
	}
	else
	{
   ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Heading*RAD2DEG,4);  
	}
	
	if(ZF_Flag)
	{
   Tra_Data[0]= GGA_Data[0];
   Tra_Data[1]= GGA_Data[1];
   Tra_Data[2]= GGA_Data[2];
   Tra_Data[3]= GGA_Data[3];
   Tra_Data[4]= GGA_Data[4];
	}
	else                                 //此处不合理，-102.2==>-102 所以应该是动态....
	{
   Tra_Data[0]= '-';
   Tra_Data[1]= GGA_Data[0];
   Tra_Data[2]= GGA_Data[1];
   Tra_Data[3]= GGA_Data[2];
   Tra_Data[4]= GGA_Data[3];
  }
//----------9-----6------------------
//ddmmyy 由ProcessGPRMC处理
//----------------------------------
  ProcessITOA(GPRMCData.Day);
  Dat_Data[0]  = GGA_Data[0];  //0
  Dat_Data[1]  = GGA_Data[1];  //1

  ProcessITOA(GPRMCData.Month);
  Dat_Data[2]  = GGA_Data[0];	//2
  Dat_Data[3] = GGA_Data[1];	//3
 
  ProcessITOA(GPRMCData.Year-2000);
  Dat_Data[4] = GGA_Data[0];	//4
  Dat_Data[5] = GGA_Data[1];	//5
//---------10-----3------------------
// 磁场变化值			   x.x
//---------------------------------
  if(GPRMCData.MagVarValid)
	{
   ProcessSFTOA(GPRMCData.MagVar,3); 
   Mag_Data[0]= GGA_Data[0];
   Mag_Data[1]= GGA_Data[1];
   Mag_Data[2]= GGA_Data[2];
   Mag_Data[3]= GGA_Data[3];
	}
	else
	{
   Mag_Data[0]= '0';
   Mag_Data[1]= '.';
   Mag_Data[2]= '0';
   Mag_Data[3]= '0';
  }
//---------11-----3------------------
//Magnetic West East 由ProcessGGA处理
//-----------------------------------
 //------------------------------------
 //  BAngle_Data
 //------------------------------------ 
  if(GINavResult.Acc_D_Angle<0)
	{
		MisAngle=GINavResult.Acc_D_Angle+360;
	}		
	else
	{
	  MisAngle=GINavResult.Acc_D_Angle;
	}
	
  ZF_Flag=ProcessSFTOA(MisAngle,4);
	
  if(ZF_Flag==1)
  {
   BAngle_Data[0] = GGA_Data[0];
   BAngle_Data[1] = GGA_Data[1];
   BAngle_Data[2] = GGA_Data[2];
   BAngle_Data[3] = GGA_Data[3];
   BAngle_Data[4] = GGA_Data[4];  
  }
  else
  {
   BAngle_Data[0] = '-';
   BAngle_Data[1] = GGA_Data[0];
   BAngle_Data[2] = GGA_Data[1];
   BAngle_Data[3] = GGA_Data[2];
   BAngle_Data[4] = GGA_Data[3];   
  }
//----------12----1------------------
//ModeIndictor   
//----------------------------------
  Mod_Data[0] = GPRMCData.ModeIn;
//----------13----1------------------
//*  				 由ProcessGGA处理
//----------------------------------
  TEN_Data[0]='*';
//----------14----1------------------
//CheckSum
//----------------------------------

//----------15----1------------------
//回车换行			 由ProcessGGA处理
//----------------------------------
	
	
	
	
 //----------0--------6---------------
 //$GPRMC			 0-5,6
 //-----------------------------------

  for(i=0;i<6;i++)		          
  TDM_TX_Data[i] = Hea_Data[i];

  TDM_TX_Data[6] = ',';
 
 //----------1--------9---------------
 //UTC hhmmss.ss    7-15,16
 //-----------------------------------
  for(i=0;i<9;i++)		          
  TDM_TX_Data[7+i] = UTC_Data[i];

  TDM_TX_Data[16] = ',';

 //----------2--------1---------------
 //DataValid         17,18
 //-----------------------------------    
  TDM_TX_Data[17] = Vad_Data[0];
  TDM_TX_Data[18] = ',';

 //----------3--------10---------------
 //维度 ddmm.mmmmm    19-30,31
 //-----------------------------------
  for(i=0;i<10;i++)		          
  TDM_TX_Data[19+i] = Lat_Data[i];

  TDM_TX_Data[29] = ',';

  
 //----------4--------1---------------
 //维度 N S           30,31
 //-----------------------------------
  TDM_TX_Data[30] = SNth_Data[0];
  TDM_TX_Data[31] = ',';


 //----------5--------11---------------
 //经度dddmm.mmmmmmmmm     34-46,47
 //-----------------------------------
  for(i=0;i<11;i++)		          
  TDM_TX_Data[32+i] = Lon_Data[i];

  TDM_TX_Data[43] = ',';
 
 //----------6-----------1---------------
 //东西 W E            48,49
 //-----------------------------------
  TDM_TX_Data[44] = WEst_Data[0];
  TDM_TX_Data[45] = ',';


 //----------7-----------5---------------
 //Speed               50-53,54
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[46+i] = Spd_Data[i];
 
  TDM_TX_Data[50] = ',';

 //----------8-----------5---------------
 //Track_Mode          55-59,60
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[51+i] = Tra_Data[i];
	

 //-----------------------------------------
  TDM_TX_Data[56] = ',';
 //----------9-----------6---------------
 //Data                61-66,67
 //-----------------------------------
  for(i=0;i<6;i++)		          
  TDM_TX_Data[57+i] = Dat_Data[i];
 
  TDM_TX_Data[63] = ',';

 //----------10-----------3---------------
 //Mag_Data             68-70,71
 //-----------------------------------

  for(i=0;i<5;i++)		          
  TDM_TX_Data[64+i] = BAngle_Data[i];

 
  TDM_TX_Data[69] = ',';

 //----------11-----------1---------------
 //东西 W E            72,73
 //-----------------------------------
  if(GPRMCData.MagVarValid)
	{
	 TDM_TX_Data[70] = GPRMCData.MagWEst;
	}
	else
	{
   TDM_TX_Data[70] = '0';
  }
	
  TDM_TX_Data[71] = ',';

 //----------12---------1-----------
 //ModeIndictor   		74,75,76,77
 //----------------------------------
  TDM_TX_Data[72] = Mod_Data[0];
  TDM_TX_Data[73] = TEN_Data[0];         
  
  CheckNumData    = ProcessCheckResult(0,73);
  ProcessCheckToA(CheckNumData);
  TDM_TX_Data[74] = GGA_Data[0];	
  TDM_TX_Data[75] = GGA_Data[1];	          		
       	  
 //----------15-----------1---------------
 //回车换行			   74 75
 //-------------------------------------- 
  TDM_TX_Data[76] = 0x0D;	          
  TDM_TX_Data[77] = 0x0A;	          	
  	
 //---------------------------------------
 //启动发送
 //---------------------------------------

  RMC_TX_Max = 78;   
	GGA_UP_Num = RMC_TX_Max;
  ZDA_UP_Num = RMC_TX_Max;
}


//-------------------------------
//void  GetGPGGA(void)
//-------------------------------
void  GetGPZDA(void)
{
static  uint8_t i;
	
uint8_t  Hea_Data[6];
uint8_t  UTC_Data[10];
uint8_t  UDate_Data[2];         //
uint8_t  UMont_Data[2];        //
uint8_t  UYear_Data[4];        //
uint8_t  UZone_Data[2];        //
uint8_t  UMine_Data[2];        //
	
uint8_t  CheckNumData;
	
	
 if(GPZDAData.GpZDA_PN_Kind==1)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'P';
  Hea_Data[3] = 'Z';
  Hea_Data[4] = 'D';
  Hea_Data[5] = 'A';
 }
 else if(GPZDAData.GpZDA_PN_Kind==2)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'L';
  Hea_Data[3] = 'Z';
  Hea_Data[4] = 'D';
  Hea_Data[5] = 'Z';  
 }
 else if(GPZDAData.GpZDA_PN_Kind==3)
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'B';
  Hea_Data[2] = 'D';
  Hea_Data[3] = 'Z';
  Hea_Data[4] = 'D';
  Hea_Data[5] = 'Z';  
 }
 else 
 {
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'N';
  Hea_Data[3] = 'Z';
  Hea_Data[4] = 'D';
  Hea_Data[5] = 'A';  
 }
 
 //-----------------------------------------
 //
 //-----------------------------------------
  ProcessITOA(GPZDAData.Hour);
  UTC_Data[0]  = GGA_Data[0];    //0
  UTC_Data[1]  = GGA_Data[1];    //1


  ProcessITOA(GPZDAData.Minute);
  UTC_Data[2]  = GGA_Data[0];	//2
  UTC_Data[3] = GGA_Data[1];	//3
 
  

  ProcessITOA(GPZDAData.Second);
  UTC_Data[4] = GGA_Data[0];	//4
  UTC_Data[5] = GGA_Data[1];	//5

  UTC_Data[6] = '.';	        //6


  ProcessITOA(GPZDAData.MSecond);

  UTC_Data[7] = GGA_Data[0];	//7
  UTC_Data[8] = GGA_Data[1];	//8

 //-----------------------------------------
 //
 //-----------------------------------------
 
  ProcessITOA(GPZDAData.Day);
  UDate_Data[0]  = GGA_Data[0];   //0
  UDate_Data[1]  = GGA_Data[1];   //1


  ProcessITOA(GPZDAData.Month);
  UMont_Data[0]  = GGA_Data[0];	  //2
  UMont_Data[1]  = GGA_Data[1];	  //3
 
  ProcessIITOA(GPZDAData.Year);
  UYear_Data[0]  = GGA_Data[0];   //0
  UYear_Data[1]  = GGA_Data[1];   //1
  UYear_Data[2]  = GGA_Data[2];   //0
  UYear_Data[3]  = GGA_Data[3];   //1
	
	//-----------------------------------------
 //
 //-----------------------------------------
  ProcessITOA(GPZDAData.ZHour);
  UZone_Data[0]  = GGA_Data[0];	  //2
  UZone_Data[1]  = GGA_Data[1];	  //3
 
  ProcessITOA(GPZDAData.ZMinute);
  UMine_Data[0]  = GGA_Data[0];	  //2
  UMine_Data[1]  = GGA_Data[1];	  //3	


	//----------------------------
	//
	//----------------------------
  for(i=0;i<6;i++)		          
  TDM_TX_Data[ZDA_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[ZDA_UP_Num+6] = ',';
	
	//----------------------------
	//  UTC
	//----------------------------
  for(i=0;i<9;i++)		          
  TDM_TX_Data[ZDA_UP_Num+7+i] = UTC_Data[i];

  TDM_TX_Data[ZDA_UP_Num+16] = ',';
	
	//----------------------------
	//  
	//----------------------------
	for(i=0;i<2;i++)		          
  TDM_TX_Data[ZDA_UP_Num+17+i] = UDate_Data[i];
	
	TDM_TX_Data[ZDA_UP_Num+19] = ',';
	
  for(i=0;i<2;i++)		          
  TDM_TX_Data[ZDA_UP_Num+20+i] = UMont_Data[i];
	
	TDM_TX_Data[ZDA_UP_Num+22] = ',';
		
	for(i=0;i<4;i++)		          
  TDM_TX_Data[ZDA_UP_Num+23+i] = UYear_Data[i];
	
	TDM_TX_Data[ZDA_UP_Num+27] = ',';
	
	//----------------------------
	//
	//----------------------------
	for(i=0;i<2;i++)		          
  TDM_TX_Data[ZDA_UP_Num+28+i] = UZone_Data[i];
	
	TDM_TX_Data[ZDA_UP_Num+30] = ',';
	
  for(i=0;i<2;i++)		          
  TDM_TX_Data[ZDA_UP_Num+31+i] = UMine_Data[i];
	
	TDM_TX_Data[ZDA_UP_Num+33] = '*';
	
	
	CheckNumData    = ProcessCheckResult(ZDA_UP_Num,33);
	ProcessCheckToA(CheckNumData);
  TDM_TX_Data[ZDA_UP_Num+34] = GGA_Data[0];	
  TDM_TX_Data[ZDA_UP_Num+35] = GGA_Data[1];
	
	TDM_TX_Data[ZDA_UP_Num+36] = 0x0D;	
  TDM_TX_Data[ZDA_UP_Num+37] = 0x0A;
	
	ZDA_TX_Max=38;
	
}


//----------------------------------
//void GetAttitude(void)
//----------------------------------
void GetAngle(void)
{
uint8_t  i;

uint8_t  Hea_Data[6];         //$GPGGA    = 6              ?? 
uint8_t  INI_Data[2];         //
uint8_t  Pitch_SData[6];      //
uint8_t  Roll_SData[6];       //
uint8_t  Yaw_SData[6];        //
uint8_t  Num_SData[2];

uint8_t  ZF_Flag;
uint8_t  Mis_Num;
uint8_t  CheckNumData;
uint8_t  Oli_Num;
//------------------------------------
 //  Head
 //------------------------------------
   
  Hea_Data[0] = '$';
  Hea_Data[1] = 'G';
  Hea_Data[2] = 'P';
  Hea_Data[3] = 'A';
  Hea_Data[4] = 'T';
  Hea_Data[5] = 'T';

 //------------------------------------
 //  Pitch
 //------------------------------------
 ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Pitch*RAD2DEG,4);

  if(ZF_Flag==1)
  {
   Pitch_SData[0] = GGA_Data[0];
   Pitch_SData[1] = GGA_Data[1];
   Pitch_SData[2] = GGA_Data[2];
   Pitch_SData[3] = GGA_Data[3];
   Pitch_SData[4] = GGA_Data[4];  
  }
  else
  {
   Pitch_SData[0] = '-';
   Pitch_SData[1] = GGA_Data[0];
   Pitch_SData[2] = GGA_Data[1];
   Pitch_SData[3] = GGA_Data[2];
   Pitch_SData[4] = GGA_Data[3];   
  }
  
 //------------------------------------
 //  Roll
 //------------------------------------  
  ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Roll*RAD2DEG,4);
	
  if(ZF_Flag==1)
  {
   Roll_SData[0] = GGA_Data[0];
   Roll_SData[1] = GGA_Data[1];
   Roll_SData[2] = GGA_Data[2];
   Roll_SData[3] = GGA_Data[3];
   Roll_SData[4] = GGA_Data[4];  
  }
  else
  {
   Roll_SData[0] = '-';
   Roll_SData[1] = GGA_Data[0];
   Roll_SData[2] = GGA_Data[1];
   Roll_SData[3] = GGA_Data[2];
   Roll_SData[4] = GGA_Data[3];   
  }
	
	//------------------------------------
 //  Yaw
 //------------------------------------  
  ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Heading*RAD2DEG,4);

  if(ZF_Flag==1)
  {
   Yaw_SData[0] = GGA_Data[0];
   Yaw_SData[1] = GGA_Data[1];
   Yaw_SData[2] = GGA_Data[2];
   Yaw_SData[3] = GGA_Data[3];
   Yaw_SData[4] = GGA_Data[4];  
  }
  else
  {
   Yaw_SData[0] = '-';
   Yaw_SData[1] = GGA_Data[0];
   Yaw_SData[2] = GGA_Data[1];
   Yaw_SData[3] = GGA_Data[2];
   Yaw_SData[4] = GGA_Data[3];   
  }
	
 //  
 //------------------------------------  
	ProcessITOA(g_GINavInfo.SYSFlag);
	INI_Data[0]  = GGA_Data[0];  //0
  INI_Data[1]  = GGA_Data[1];  //1
	
//----------0--------6---------------
 //$GPATT			 0-5,6
 //-----------------------------------
  
		ATT_UP_Num=0;
	
  for(i=0;i<6;i++)		          
  TDM_TX_Data[ATT_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[ATT_UP_Num+6] = ',';
 
 //----------1--------5---------------
 //Ptich			 7-12,13
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[ATT_UP_Num+7+i] = Pitch_SData[i];

  TDM_TX_Data[ATT_UP_Num+12] = ',';
 //----------2--------1---------------
 //p  			    14,15
 //-----------------------------------
  TDM_TX_Data[ATT_UP_Num+13] = 'p';
  TDM_TX_Data[ATT_UP_Num+14] = ',';
  //---------3--------5---------------
 //Roll  		   16-20,21
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[ATT_UP_Num+15+i] = Roll_SData[i];

  TDM_TX_Data[ATT_UP_Num+20] = ',';
 //----------2--------1---------------
 //r			    22,23
 //-----------------------------------
  TDM_TX_Data[ATT_UP_Num+21] = 'r';
  TDM_TX_Data[ATT_UP_Num+22] = ',';
 //---------3--------5---------------
 //Roll  		  24-28,29
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[ATT_UP_Num+23+i] = Yaw_SData[i];

  TDM_TX_Data[ATT_UP_Num+28] = ',';
 //----------2--------1---------------
 //y			    30,31
 //-----------------------------------
  TDM_TX_Data[ATT_UP_Num+29] = 'y';
  TDM_TX_Data[ATT_UP_Num+30] = ',';

 //----------2--------1---------------
 //Soft_Version
 //-----------------------------------
	ProcessVTOA(SVersionH);
	for(i=0;i<4;i++)
	{
	 TDM_TX_Data[ATT_UP_Num+31+i] = GGA_Data[i];	 
	}
		
	ProcessVTOA(SVersionL);
	for(i=0;i<4;i++)
	{
	 TDM_TX_Data[ATT_UP_Num+35+i] = GGA_Data[i];	 
	}
	
	TDM_TX_Data[ATT_UP_Num+39] = ',';
	
	TDM_TX_Data[ATT_UP_Num+40] = 's';
  TDM_TX_Data[ATT_UP_Num+41] = ',';
	
	
 //----------2--------1---------------
 //96bit ID
 //-----------------------------------

  for(i=0;i<6;i++)
  {		 	
	 TDM_TX_Data[ATT_UP_Num+41+1+i*4]  = CPU_ID_Data[0+i*4];
	 TDM_TX_Data[ATT_UP_Num+41+2+i*4]  = CPU_ID_Data[1+i*4];
	 TDM_TX_Data[ATT_UP_Num+41+3+i*4]  = CPU_ID_Data[2+i*4];
	 TDM_TX_Data[ATT_UP_Num+41+4+i*4]  = CPU_ID_Data[3+i*4];
  }

	
	TDM_TX_Data[ATT_UP_Num+66] = ',';
	
	TDM_TX_Data[ATT_UP_Num+67] = 'I';
	TDM_TX_Data[ATT_UP_Num+68] = 'D';
  TDM_TX_Data[ATT_UP_Num+69] = ',';
	
		
 //----------2--------1---------------
 // 惯性导航打开与否
 //-----------------------------------
	
	
  TDM_TX_Data[ATT_UP_Num+70] = 0x30+INS_DGet_Flag;
	
		
	TDM_TX_Data[ATT_UP_Num+71] = ',';
	TDM_TX_Data[ATT_UP_Num+72] = 'I';
	TDM_TX_Data[ATT_UP_Num+73] = 'N';
	TDM_TX_Data[ATT_UP_Num+74] = 'S';
  TDM_TX_Data[ATT_UP_Num+75] = ',';
	
	TDM_TX_Data[ATT_UP_Num+76] = '4';
	TDM_TX_Data[ATT_UP_Num+77] = '0';
	TDM_TX_Data[ATT_UP_Num+78] = '1';
  TDM_TX_Data[ATT_UP_Num+79] = ',';
 //----------2--------1---------------
 //			    32,33
 //-----------------------------------
  TDM_TX_Data[ATT_UP_Num+80] = INI_Data[0];
  TDM_TX_Data[ATT_UP_Num+81] = INI_Data[1];	
	TDM_TX_Data[ATT_UP_Num+82] = ',';
	
	//-----------------------------------------------
	//
	//-----------------------------------------------
 
	 Mis_Num = GINavResult.Acc_B_Num/2;
	 
	if(Mis_Num>=7)
	{	
	 Mis_Num=Mis_Num-1;
	}
	else
	{
	 Mis_Num=Mis_Num;
	}
	
	if(Mis_Num>=9)
		Mis_Num=9;
	
	TDM_TX_Data[ATT_UP_Num+83] = Mis_Num+0x30;
	TDM_TX_Data[ATT_UP_Num+84] = ',';	
	
	//-----------------------------------------------
	//
	//-----------------------------------------------

	TDM_TX_Data[GSV_UP_Num+85]= SGPS_DGet_Flag+0x30;   
	TDM_TX_Data[ATT_UP_Num+86] = ',';
	//-----------------------------------------------
	//
	//-----------------------------------------------
  if(BD_GGet_Flag)	
	{
	 TDM_TX_Data[ATT_UP_Num+87] = 'B';
	}
	else
	{
	 TDM_TX_Data[ATT_UP_Num+87] = 'G';
	}
	
	TDM_TX_Data[ATT_UP_Num+88] = ',';
	//-----------------------------------------------
	//
	//-----------------------------------------------
	if(User_Kind==3)
	{
	 TDM_TX_Data[ATT_UP_Num+89] = 'H';
	 TDM_TX_Data[ATT_UP_Num+90] = 'X';
	}
	else
	{
	 TDM_TX_Data[ATT_UP_Num+89] = 'A';
	 TDM_TX_Data[ATT_UP_Num+90] = 'U';
	}
	
	TDM_TX_Data[ATT_UP_Num+91] = ',';
	
	TDM_TX_Data[ATT_UP_Num+92] = GINavResult.StaticFlag+0x30;	
	TDM_TX_Data[ATT_UP_Num+93] = ',';
	
	TDM_TX_Data[ATT_UP_Num+94] = User_Kind+0x30;	
	TDM_TX_Data[ATT_UP_Num+95] = ',';
	
	TDM_TX_Data[ATT_UP_Num+96] = g_GINavInfo.Install_Ini_Flag+0x30;	
	TDM_TX_Data[ATT_UP_Num+97] = ',';
	
	TDM_TX_Data[ATT_UP_Num+98] = RSE_DGet_Flag+0x30;	
	TDM_TX_Data[ATT_UP_Num+99] = ',';
		
	TDM_TX_Data[ATT_UP_Num+100] = 'F';
	TDM_TX_Data[ATT_UP_Num+101] = ',';
	
	TDM_TX_Data[ATT_UP_Num+102] = ANG_Lock_Flag+0x30;     //0： 没有锁定（默认）   1：锁定
	TDM_TX_Data[ATT_UP_Num+103] = ',';
	
	TDM_TX_Data[ATT_UP_Num+104] = GINavResult.UBI_Kind_Off+0x30;       //
	TDM_TX_Data[ATT_UP_Num+105] = ',';
	
	TDM_TX_Data[ATT_UP_Num+106] = GINavResult.M_City_Flag+0x30;        //
	TDM_TX_Data[ATT_UP_Num+107] = ',';
		
	Oli_Num=ProcessZLSFTOA(GINavResult.OliData/1000.0,3);	
	
  for(i=0;i<Oli_Num;i++)
  {		 	
	 TDM_TX_Data[ATT_UP_Num+108+i]  = GGA_Data[i];
	}

	TDM_TX_Data[ATT_UP_Num+Oli_Num+108] = '*';	


	CheckNumData    = ProcessCheckResult(ATT_UP_Num,Oli_Num+108);
	ProcessCheckToA(CheckNumData);
  TDM_TX_Data[ATT_UP_Num+Oli_Num+109] = GGA_Data[0];	
  TDM_TX_Data[ATT_UP_Num+Oli_Num+110] = GGA_Data[1];
	
	TDM_TX_Data[ATT_UP_Num+Oli_Num+111] = 0x0D;	
  TDM_TX_Data[ATT_UP_Num+Oli_Num+112] = 0x0A;
	
	
	ATT_TX_Max   = Oli_Num+113;
  GSV_UP_Num   = ATT_UP_Num+ATT_TX_Max;
	GIRMC_UP_Num = ATT_UP_Num+ATT_TX_Max;
	
}


//--------------------------------
//void GetRMCData(void)
//--------------------------------
void GetRMCData(void)
{
 static  uint8_t i;
	
	if(ATT_DGet_Flag==0)
	{
		GIRMC_UP_Num=0;
	}
	
	 for(i=0;i<GGARMC_Length;i++)		
	{
    TDM_TX_Data[GIRMC_UP_Num+i]=m_RMCpData[i];
	}


}
//--------------------------------------
//void GetAngleDis(void)
//--------------------------------------
void GetAngleDis(void)
{
static   uint8_t i;
uint8_t  CheckNumData;
	//----------------Debug----------------------------------------
 
uint8_t  Hea_Data[6];          //	
uint8_t  Num_Data[2];	        

	
uint8_t	 AAngle_Data[5];
uint8_t	 BAngle_Data[5];
uint8_t	 MAngle_Data[5];
uint8_t	 Tra_Data[5];	
	
uint8_t	 ZF_Flag;
//------------------------------
//
//------------------------------
	Hea_Data[0] = '$';
  Hea_Data[1] = 'D';
  Hea_Data[2] = 'E';
  Hea_Data[3] = 'B';
  Hea_Data[4] = 'U';
  Hea_Data[5] = 'G';
	
	
//------------------------------------
//
//------------------------------------


//------------------------------------
 //  MAngle_Data
 //------------------------------------
   ZF_Flag=ProcessSFTOA(MisAngleData,4);

  if(ZF_Flag==1)
  {
   MAngle_Data[0] = GGA_Data[0];
   MAngle_Data[1] = GGA_Data[1];
   MAngle_Data[2] = GGA_Data[2];
   MAngle_Data[3] = GGA_Data[3];
   MAngle_Data[4] = GGA_Data[4];  
  }
  else
  {
   MAngle_Data[0] = '-';
   MAngle_Data[1] = GGA_Data[0];
   MAngle_Data[2] = GGA_Data[1];
   MAngle_Data[3] = GGA_Data[2];
   MAngle_Data[4] = GGA_Data[3];   
  }
	

 //------------------------------------
 //  AAngle_Data
 //------------------------------------
    ZF_Flag=ProcessSFTOA(GINavResult.Acc_C_Angle,4);

  if(ZF_Flag==1)
  {
   AAngle_Data[0] = GGA_Data[0];
   AAngle_Data[1] = GGA_Data[1];
   AAngle_Data[2] = GGA_Data[2];
   AAngle_Data[3] = GGA_Data[3];
   AAngle_Data[4] = GGA_Data[4];  
  }
  else
  {
   AAngle_Data[0] = '-';
   AAngle_Data[1] = GGA_Data[0];
   AAngle_Data[2] = GGA_Data[1];
   AAngle_Data[3] = GGA_Data[2];
   AAngle_Data[4] = GGA_Data[3];   
  }
  
 //------------------------------------
 //  BAngle_Data
 //------------------------------------  
  ZF_Flag=ProcessSFTOA(GINavResult.Acc_D_Angle,4);
	
  if(ZF_Flag==1)
  {
   BAngle_Data[0] = GGA_Data[0];
   BAngle_Data[1] = GGA_Data[1];
   BAngle_Data[2] = GGA_Data[2];
   BAngle_Data[3] = GGA_Data[3];
   BAngle_Data[4] = GGA_Data[4];  
  }
  else
  {
   BAngle_Data[0] = '-';
   BAngle_Data[1] = GGA_Data[0];
   BAngle_Data[2] = GGA_Data[1];
   BAngle_Data[3] = GGA_Data[2];
   BAngle_Data[4] = GGA_Data[3];   
  }
	
	//-----------------------------
	//Num
  //-----------------------------
	ProcessITOA(GINavResult.Acc_B_Num);
	Num_Data[0]  = GGA_Data[0];  //0
  Num_Data[1]  = GGA_Data[1];  //1
	
	
 //----------0--------1---------------
 //$Debug		 0-5,6
 //-----------------------------------
  for(i=0;i<6;i++)		          
  TDM_TX_Data[GSV_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[GSV_UP_Num+6] = ',';
	
 //----------0--------2---------------
 //
 //-----------------------------------
	TDM_TX_Data[GSV_UP_Num+7]  = 'K';
	TDM_TX_Data[GSV_UP_Num+8]  = 'I';
	TDM_TX_Data[GSV_UP_Num+9]  = 'N';
	TDM_TX_Data[GSV_UP_Num+10] = 'D';
  TDM_TX_Data[GSV_UP_Num+11] = ',';
		
  TDM_TX_Data[GSV_UP_Num+12] = 0x30+ANG_Kind_Flag;         //0:自适应版本 1：固定版本
  TDM_TX_Data[GSV_UP_Num+13] = ',';
	
	TDM_TX_Data[GSV_UP_Num+14] = 0x30+IMU_Kind;              // 固定版本的坐标系...
	TDM_TX_Data[GSV_UP_Num+15] = ',';
	
	TDM_TX_Data[GSV_UP_Num+16] = 0x30+GINavResult.Ini0_Kind; //自适应版本的坐标系
	TDM_TX_Data[GSV_UP_Num+17] = ',';
			
 //-----------1------------------------
 //
 //-----------------------------------

  TDM_TX_Data[GSV_UP_Num+18] = 'M';
	TDM_TX_Data[GSV_UP_Num+19] = 'E';
	TDM_TX_Data[GSV_UP_Num+20] = 'M';
	TDM_TX_Data[GSV_UP_Num+21] = ',';
	
	TDM_TX_Data[GSV_UP_Num+22] = 0x30+ANG_Lock_Flag;     //0： 没有锁定（默认）   1：锁定
  TDM_TX_Data[GSV_UP_Num+23] = ',';
	
  TDM_TX_Data[GSV_UP_Num+24] = 0x30+ANG_DGet_Flag;     //0： 没有保存安装角     1：有安装角
  TDM_TX_Data[GSV_UP_Num+25] = ',';
	
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GSV_UP_Num+26+i] = MAngle_Data[i];      // 存储安装角

  TDM_TX_Data[GSV_UP_Num+31] = ',';
	
 //----------0--------3---------------
 //View_Data		10-11,12
 //-----------------------------------
  TDM_TX_Data[GSV_UP_Num+32] = 'N';
	TDM_TX_Data[GSV_UP_Num+33] = 'E';
	TDM_TX_Data[GSV_UP_Num+34] = 'W';
	TDM_TX_Data[GSV_UP_Num+35] = ',';
	
  TDM_TX_Data[GSV_UP_Num+36] = 0x30+ANG_FGet_Flag;     //0:  新的安装角无须替换旧安装角，1：新安装角替换旧安装角

  TDM_TX_Data[GSV_UP_Num+37] = ',';
	
	for(i=0;i<2;i++)		          
  TDM_TX_Data[GSV_UP_Num+38+i] = Num_Data[i];          //新找到安装角的个数

  TDM_TX_Data[GSV_UP_Num+40] = ',';
	
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GSV_UP_Num+41+i] = BAngle_Data[i];       //新找到安装角

  TDM_TX_Data[GSV_UP_Num+46] =',';
	
  TDM_TX_Data[GSV_UP_Num+47] = 0x30+GINavResult.Ini_Flag; 	
	TDM_TX_Data[GSV_UP_Num+48] =',';
//-----------------------------------	

  TDM_TX_Data[GSV_UP_Num+49] = 'R';
	TDM_TX_Data[GSV_UP_Num+50] = 'S';
	TDM_TX_Data[GSV_UP_Num+51] = 'E';
	TDM_TX_Data[GSV_UP_Num+52] = ',';
	
	
  ProcessLatToA(Lat_Back*RAD2DEG);     //10-19;	 		

  for(i=0;i<10;i++)		          
  TDM_TX_Data[GSV_UP_Num+53+i] = GGA_Data[i];       //新找到安装角
	
	TDM_TX_Data[GSV_UP_Num+63] =',';
	
	
	//-----------------------------------	
  ProcessLongToA(Lon_Back*RAD2DEG);     	 		
 	
  for(i=0;i<11;i++)		          
  TDM_TX_Data[GSV_UP_Num+64+i] = GGA_Data[i];       //新找到安装角
	
	TDM_TX_Data[GSV_UP_Num+75] =',';
	
	
	TDM_TX_Data[GSV_UP_Num+76] = 'I';
	TDM_TX_Data[GSV_UP_Num+77] = 'N';
	TDM_TX_Data[GSV_UP_Num+78] = 'I';
	TDM_TX_Data[GSV_UP_Num+79] = ',';
	
 //------------------------------------
 // 
 //------------------------------------  
	TDM_TX_Data[GSV_UP_Num+80] =Ini0_KindNew+0X30;
	TDM_TX_Data[GSV_UP_Num+81] =',';
 //------------------------------------
 // 
 //------------------------------------  
  ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Heading*RAD2DEG,4);
	
  if(ZF_Flag==1)
  {
   BAngle_Data[0] = GGA_Data[0];
   BAngle_Data[1] = GGA_Data[1];
   BAngle_Data[2] = GGA_Data[2];
   BAngle_Data[3] = GGA_Data[3];
   BAngle_Data[4] = GGA_Data[4];  
  }
  else
  {
   BAngle_Data[0] = '-';
   BAngle_Data[1] = GGA_Data[0];
   BAngle_Data[2] = GGA_Data[1];
   BAngle_Data[3] = GGA_Data[2];
   BAngle_Data[4] = GGA_Data[3];   
  }
	
	
	for(i=0;i<5;i++)		          
  TDM_TX_Data[GSV_UP_Num+82+i] = BAngle_Data[i];       //新找到安装角
	
	TDM_TX_Data[GSV_UP_Num+87] =',';
	
	//-----------------------------------

	TDM_TX_Data[GSV_UP_Num+88] = RSE_DGet_Flag+0x30;
	
	TDM_TX_Data[GSV_UP_Num+89] ='*';



	
	CheckNumData    = ProcessCheckResult(GSV_UP_Num,89);
	ProcessCheckToA(CheckNumData);
  TDM_TX_Data[GSV_UP_Num+90] = GGA_Data[0];	
  TDM_TX_Data[GSV_UP_Num+91] = GGA_Data[1];
	
	TDM_TX_Data[GSV_UP_Num+92] = 0x0D;	
  TDM_TX_Data[GSV_UP_Num+93] = 0x0A;
  
	GSV_TX_Max=94;

	GIRMC_UP_Num = GSV_UP_Num+GSV_TX_Max;
}

//----------------------------
//void Send-GnssData(void)
//----------------------------
void Send_GnssData(void)
{
	uint8_t RunFlag=0;
	
	if(Debug_Flag)
	{
   RunFlag= Gnss_Up_Flag&&GPS_Send_OK&&ATT_Send_OK;
  } 
	else
	{
   RunFlag = Gnss_Up_Flag&&ATT_Send_OK;
  }
	
	if(RunFlag)
	{
		Gnss_Up_Flag   =  0;
	  DMA_Send_Kind  =  3;                            //数据类型
		DMA_Tx_Num     =  0;
		//------------------------------------
		//      RMC VTG GGA
		//------------------------------------	
		if(Rmc_Vtg_Flag==1)
	  {
			DMA_Tx_Num = RMC_TX_Max;			
		}
	  else
		{
      DMA_Tx_Num =  VTG_TX_Max;		
    }		
	
		//------------------------------------
		//      GGA
		//------------------------------------	
     
		 			 
		 DMA_Tx_Num = DMA_Tx_Num + GGA_TX_Max; 
		
 	

			if(ZDA_DGet_Flag==1)
			{
       DMA_Tx_Num = DMA_Tx_Num + ZDA_TX_Max;
			}

		
	  //------------------------------------
		//      Send
		//------------------------------------
		if(DMA_Tx_Num>0)		
		{
	   USART_DMA_TX_Configuration(TDM_TX_Data);        //DMA??
	 
	   SendDriver(); 
		}
	
	}
}
//-----------------------------
//void SendGnssData(void)
//-----------------------------
void Pack_GnssData(void)
{
	uint8_t Run_Flag=0;
	
  Run_Flag = Gnss_Get_Flag&&ATT_Send_OK;   
  
	if(Run_Flag)                    //并且获得组合导航结果
	{
		Gnss_Get_Flag     = 0;               //
		GNSS_Send_OK_Flag = 1;

		
	  if(Rmc_Vtg_Flag==1)
		{			
		 GetGPRMC();                    //组织RMC协议数据
		} 
    else
		{
     GetGPVTG();                   //组织VTG协议数据
    }		    
	
    GetLGPGGA();                   //组织GGA协议数据
		
		
		if(ZDA_DGet_Flag)
		{
		 GetGPZDA();                  //
    }

		Gnss_Up_Flag=1;
		
	}
}


//-----------------------------
//void Pack_Debug(void)
//-----------------------------
void Pack_Send_Debug(void)
{
	if(PackDebug_Flag)
	{
		PackDebug_Flag=0;
		GSV_UP_Num    =0;
		DMA_Tx_Num    =0;
		
		if(ATT_DGet_Flag||Debug_Flag)
		{
		 GetAngle();
		
		 if(Debug_Flag)
		 {
		  GetAngleDis();
		 }
		}
			
		if(RMC_Back_Flag)
		GetRMCData();
		
		//------------------------------------
		//      ATT
		//------------------------------------		
	   if(ATT_DGet_Flag||Debug_Flag)
		 {
			 if(Debug_Flag)
		   {
        DMA_Tx_Num  = ATT_TX_Max+GSV_TX_Max;
		   }
       else
			 {
			  DMA_Tx_Num  = ATT_TX_Max;
			 }				 
		 }
      else
		 {
       DMA_Tx_Num  = 0;
     }			 
		 
		//------------------------------------
		//      GSV
		//------------------------------------		
	   if(RMC_Back_Flag&&(Debug_Flag==0))
		 {
        DMA_Tx_Num  =  DMA_Tx_Num+GGARMC_Length;
		 }
		 
		 //------------------------------------
		 if(DMA_Tx_Num>0)
		 {				
			 USART_DMA_TX_Configuration(TDM_TX_Data);        //DMA使能
			 
			 DMA_Send_Kind = 5;
			 
			 SendDriver(); 
     }
		 else                                               //如果不发送ATT+GSV，则置位ATT_Send_OK，用于驱动GNSS
		 {
       ATT_Send_OK=1;
     }
   }	
}

//----------------------------
//void Send-GnssData(void)
//----------------------------
void Send_GSAData(void)
{
	
if((GNSS_Send_OK==1)&&(Debug_Flag==0)&&(GSA_Length>0)&&(GSV_DGet_Flag))
	{
		GNSS_Send_OK   =  0;
	  DMA_Send_Kind  =  4;                            //Кэ?ЭАаРН
		
		DMA_Tx_Num   = GSA_Length;
	 
		
    if(DMA_Tx_Num>0)
		{			
		 USART_DMA_TX_Configuration(m_GSPDataB);
		
	   SendDriver(); 
		}
  }
}

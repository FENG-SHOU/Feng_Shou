#include "commdata.h"
#include "GetIMU.h"
#include "delay.h"
#include "GetAcc.h"
#include "GetGyr.h"
#include "stmflash.h"




void HandleUserData(uint8_t UerData)
{	
	 if(Up_On_Flag==1)
	 {
    Command_Handle(UerData);
	 }
		
	  HandlePassOn(UerData);
	  HandleUpdata(UerData);		
	  HandleKeyOn(UerData);
  
	 
	   //-----------------------------------
	   if((GINavResult.GpsHighFlag==1)||(SGPS_DGet_Flag==0))  //保证基站一帧数据可以发送完毕之后，才停止基站....
	   {
	    USART_SendData(USART2, UerData);
	   }
	 
	 	 InPutDiffNum++;
	   if(InPutDiffNum>=200)
		  InPutDiffNum=200;
	 

}


//------------------------------------------------
//void ProcessITOA(uint8_t GGAIData,uint8_t bit)
//------------------------------------------------
void ProcessVTOA(uint16_t VData)
{									
 static  uint8_t  i;
  			     
	GGA_Data[0] = (VData)/1000;	                                    //??    
  GGA_Data[1] = (VData)/100  - ((uint16_t)GGA_Data[0])*10;        //??        
	GGA_Data[2] = (VData)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //??        	
	GGA_Data[3] = (VData)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[2]*10) ;  //??        	
	
    for(i=0;i<4;i++)
    {
    GGA_Data[i] = GGA_Data[i] + 0x30;   
    }
}

void Com2SendData(void)
{ 
	if((COM_Flag==1)&&(Flash_Wing_Flag==0))
	{
		COM_Flag=0;		
				
	 DMA_Tx_Num=Command_Tx_Max;	
		
	 if(DMA_Tx_Num>0)
	 {		 
	  USART_DMA_TX_Configuration(TDM_TX_Data);        //DMA??
		
	  SendDriver();
	 }
	 
	}	
}
//--------------------------------
//uint16_t GetCrcData(void)
//--------------------------------
uint16_t GetCrcData(void)
{
 uint16_t Check_num;
 uint8_t i;
  
  Check_num = 0;
  Check_num = Check_num+TDM_TX_Data[0];
  Check_num = Check_num+TDM_TX_Data[1];

    for(i=2;i<TDM_TX_Data[1]-2;i=i+2)
   {
    Check_num = Check_num+(int16_t)(TDM_TX_Data[i]*256);
   }
   for(i=3;i<TDM_TX_Data[1]-2;i=i+2)
   {
    Check_num = Check_num+TDM_TX_Data[i];
   }

  TDM_TX_Data[TDM_TX_Data[1]-2] = Check_num>>0x08;
  TDM_TX_Data[TDM_TX_Data[1]-1] = Check_num;

  return  Check_num;
}


void SendData(void)
{
   uint16_t crcdata=0;
   uint8_t  Command_Valid_Flag=0;


 if((AHRSComFlag)&&(Flash_Wing_Flag==0))          //?????&&????&&???
 {		 
			AHRSComFlag=0;
	 
			if(Data_Com_Flag==1)
			{			 
			 if(Data_Sort==0x01)                  //  0x01
			 {
			 Command_Valid_Flag = 1;

			 TDM_TX_Data[0]= 0x01;
			 TDM_TX_Data[1]= 0x18;
			
			 TDM_TX_Data[2]= Gyr_Y>>0x08;
			 TDM_TX_Data[3]= Gyr_Y;

			 TDM_TX_Data[4]= Gyr_X>>0x08;
			 TDM_TX_Data[5]= Gyr_X;

			 TDM_TX_Data[6]= Gyr_Z>>0x08;
			 TDM_TX_Data[7]= Gyr_Z;

			 TDM_TX_Data[8]= ACC_X>>0x08;
			 TDM_TX_Data[9]= ACC_X;
			
			 TDM_TX_Data[10]= ACC_Y>>0x08;
			 TDM_TX_Data[11]= ACC_Y;

			 TDM_TX_Data[12]= ACC_Z>>0x08;
			 TDM_TX_Data[13]= ACC_Z;
			 
			 TDM_TX_Data[14]= 0;
			 TDM_TX_Data[15]= 0;

			 TDM_TX_Data[16]= 0;
			 TDM_TX_Data[17]= 0;
			
			 TDM_TX_Data[18]= 0;
			 TDM_TX_Data[19]= 0;

			 TDM_TX_Data[20]= 0;
			 TDM_TX_Data[21]= 0;

			}
		else if(Data_Sort==0x06)                 //0x06
			{
			 Command_Valid_Flag = 1;

			 TDM_TX_Data[0] = 0x06;
			 TDM_TX_Data[1] = 30;
			
			 TDM_TX_Data[2] = intRollGyr>>0x08;
			 TDM_TX_Data[3] = intRollGyr;

			 TDM_TX_Data[4] = intPitchGyr>>0x08;
			 TDM_TX_Data[5] = intPitchGyr;

			 TDM_TX_Data[6] = intYawGyr>>0x08;
			 TDM_TX_Data[7] = intYawGyr;

			 TDM_TX_Data[8] = intPitchAcc>>0x08;
			 TDM_TX_Data[9] = intPitchAcc;

			 TDM_TX_Data[10]= intRollAcc>>0x08;
			 TDM_TX_Data[11]= intRollAcc;

			 TDM_TX_Data[12]= intAliAcc>>0x08;
			 TDM_TX_Data[13]= intAliAcc;

			 TDM_TX_Data[14]= 0;
			 TDM_TX_Data[15]= 0;

			 TDM_TX_Data[16]= 0;
			 TDM_TX_Data[17]= 0;

			 TDM_TX_Data[18]= 0;
			 TDM_TX_Data[19]= 0; 	   
											
			 TDM_TX_Data[20]= intPitchAngle>>0x08;
			 TDM_TX_Data[21]= intPitchAngle;

			 TDM_TX_Data[22]= intRollAngle>>0x08;
			 TDM_TX_Data[23]= intRollAngle;

			 TDM_TX_Data[24]= intYawAngle>>0x08;
			 TDM_TX_Data[25]= intYawAngle;

			 TDM_TX_Data[26]= 0;
			 TDM_TX_Data[27]= 0;    
			
			 TDM_TX_Data[28]= 0;
			 TDM_TX_Data[29]= 0; 		    
			
			}
			
			Command_Tx_Max= TDM_TX_Data[1];  //发送数据个数
			
			if(Command_Valid_Flag)
			{
				Command_Valid_Flag=0;
				
			 crcdata=GetCrcData();

			 TDM_TX_Data[TDM_TX_Data[1]-2]=crcdata>>0x08;
			 TDM_TX_Data[TDM_TX_Data[1]-1]=crcdata; 
			
			 COM_Flag=1;   
			}	
		}
		else if(Data_Com_Flag==2)      //命令
		{
			Data_Com_Flag=1;             //下次进入数据

				
			if(Comm_Sort==0x08)             //08 启动陀螺仪寻偏置
			{
			Command_Valid_Flag = 1;


			TDM_TX_Data[0] = 0x08;
			TDM_TX_Data[1] = 0x04;

			TDM_TX_Data[2] = 0x00;
			TDM_TX_Data[3] = 0x00;
			}
			else if(Comm_Sort==0x10)          //10   查询软件版本号
			{
			Command_Valid_Flag = 1;

			TDM_TX_Data[0] = 0x10;
			TDM_TX_Data[1] = 0x06;

			TDM_TX_Data[2] = HVersionH>>0x08;
			TDM_TX_Data[3] = HVersionH;

			TDM_TX_Data[4] = 0x00;
			TDM_TX_Data[5] = 0x00;	
			}
			else if(Comm_Sort==0x11)           //11   查询序列号
			{
			Command_Valid_Flag = 1;

			TDM_TX_Data[0] = 0x11;
			TDM_TX_Data[1] = 0x06;

			TDM_TX_Data[2] = HVersionL>>0x08;
			TDM_TX_Data[3] = HVersionL;

			TDM_TX_Data[4] = 0x00;
			TDM_TX_Data[5] = 0x00;
			}
			else if(Comm_Sort==0x12)           //12   查询产品标识
			{
			Command_Valid_Flag = 1;

			TDM_TX_Data[0] = 0x12;
			TDM_TX_Data[1] = 0x08;

			TDM_TX_Data[2] = 0x3A;
			TDM_TX_Data[3] = 0x47;

			TDM_TX_Data[4] = 0x54;
			TDM_TX_Data[5] = 0x59;

			TDM_TX_Data[6] = 0x00;
			TDM_TX_Data[7] = 0x00;
			}

			//-----------------------------------------
			else if(Comm_Sort==0x41)          //41   查询作者
			{

				Command_Valid_Flag = 1;

				 TDM_TX_Data[0] = 0x41;
				 TDM_TX_Data[1] = 0x20;

				 TDM_TX_Data[2] = 'A';
				 TDM_TX_Data[3] = 'u';
				 TDM_TX_Data[4] = 't';
				 TDM_TX_Data[5] = 'h';
				 TDM_TX_Data[6] = 'o';
				 TDM_TX_Data[7] = 'r';
				 TDM_TX_Data[8] = ':';

				 TDM_TX_Data[9] = 'G';
				 TDM_TX_Data[10] = 'a';
				 TDM_TX_Data[11] = 'o';
				 TDM_TX_Data[12] = ' ';

				 TDM_TX_Data[13] = 'T';
				 TDM_TX_Data[14] = 'o';
				 TDM_TX_Data[15] = 'n';
				 TDM_TX_Data[16] = 'g';
				 TDM_TX_Data[17] = ' ';

				 TDM_TX_Data[18] = 'Y';
				 TDM_TX_Data[19] = 'u';
				 TDM_TX_Data[20] = 'e';

				 TDM_TX_Data[21] = ' ';

				 TDM_TX_Data[22] = '2';
				 TDM_TX_Data[23] = '0';
				 TDM_TX_Data[24] = '1';
				 TDM_TX_Data[25] = '8';

				 TDM_TX_Data[26] = '0';
				 TDM_TX_Data[27] = '5';
				 TDM_TX_Data[28] = '1';
				 TDM_TX_Data[29] = '6';
			 
			 }
				else if(Comm_Sort==0x50)           //50   查询陀螺参数
			 {
					Command_Valid_Flag = 1;

					TDM_TX_Data[0] = 0x50;
					TDM_TX_Data[1] = 0x10;

					TDM_TX_Data[2] = YGyrBias>>0x08;
					TDM_TX_Data[3] = YGyrBias;

					TDM_TX_Data[4] = XGyrBias>>0x08;
					TDM_TX_Data[5] = XGyrBias;

					TDM_TX_Data[6] = ZGyrBias>>0x08;
					TDM_TX_Data[7] = ZGyrBias;

					TDM_TX_Data[8] = YGyrScale>>0x08;
					TDM_TX_Data[9] = YGyrScale;

					TDM_TX_Data[10] = XGyrScale>>0x08;
					TDM_TX_Data[11] = XGyrScale;

					TDM_TX_Data[12] = ZGyrScale>>0x08;
					TDM_TX_Data[13] = ZGyrScale;

					TDM_TX_Data[14] = 0;
					TDM_TX_Data[15] = 0;  
			 }					
			 else if(Comm_Sort==0x51)          //51   查询加速度计参数
			 {
				Command_Valid_Flag = 1;

				TDM_TX_Data[0] = 0x51;
				TDM_TX_Data[1] = 0x2E;
				//-------------------------------
				TDM_TX_Data[2] = XAccBias>>0x08;
				TDM_TX_Data[3] = XAccBias;

				TDM_TX_Data[4] = YAccBias>>0x08;
				TDM_TX_Data[5] = YAccBias;

				TDM_TX_Data[6] = ZAccBias>>0x08;
				TDM_TX_Data[7] = ZAccBias;
				//-------------------------------
				TDM_TX_Data[8] = XAccScale>>0x08;
				TDM_TX_Data[9] = XAccScale;

				TDM_TX_Data[10] = YAccScale>>0x08;
				TDM_TX_Data[11] = YAccScale;

				TDM_TX_Data[12] = ZAccScale>>0x08;
				TDM_TX_Data[13] = ZAccScale;


				TDM_TX_Data[44] = 0;
				TDM_TX_Data[45] = 0;  
			}
			else if(Comm_Sort==0x55)          //55  ????
			{
				Command_Valid_Flag = 1;

				TDM_TX_Data[0] = 0x55;
				TDM_TX_Data[1] = 0x10;

				TDM_TX_Data[2] = HVersionH>>0x08;
				TDM_TX_Data[3] = HVersionH;

				TDM_TX_Data[4] = HVersionL>>0x08;
				TDM_TX_Data[5] = HVersionL;

				TDM_TX_Data[6] = SVersionH>>0x08;
				TDM_TX_Data[7] = SVersionH;

				TDM_TX_Data[8] = SVersionL>>0x08;
				TDM_TX_Data[9] = SVersionL;

				TDM_TX_Data[10] = HVersionS>>0x08;
				TDM_TX_Data[11] = HVersionS;

				TDM_TX_Data[14] = 0; 
				TDM_TX_Data[15] = 0; 
			}
							
			Command_Tx_Max= TDM_TX_Data[1];  //??????
			
			if(Command_Valid_Flag)
			{
				Command_Valid_Flag=0;
				
			 crcdata=GetCrcData();

			 TDM_TX_Data[TDM_TX_Data[1]-2]=crcdata>>0x08;
			 TDM_TX_Data[TDM_TX_Data[1]-1]=crcdata; 

			 COM_Flag=1;   
			}	
		}
	 }
}





//----------------------------------------
//void Command_Handle(uchar Comamnd)
//----------------------------------------
void Command_Handle(unsigned char Comamnd)
{
   static uint8_t TDM_Rx_Counter=0;
          uint8_t  Command_Sort=0;       //???????????
	
   TDM_Rx_Data[TDM_Rx_Counter]=Comamnd;
   TDM_Rx_Counter++;
     
		if(TDM_Rx_Counter==1)              //
		{
		 if(TDM_Rx_Data[0]==0x41) //
		 {
		  Command_Rx_Max = 2;
		 }
		 else if(TDM_Rx_Data[0]==0x60)  //??????
		 {
		  Command_Rx_Max = 15; 
		 }
		 else if(TDM_Rx_Data[0]==0x61)  //??????
		 {
		  Command_Rx_Max = 45; 
		 }
		 else if(TDM_Rx_Data[0]==0x65)  //??????
		 {
		  Command_Rx_Max = 15; 
		 }
		 else
		 { 
			Command_Rx_Max = 1;		
		 }     
		}//if(TDM_Rx_Counter==1)
 
   //---------------------------------

    if(TDM_Rx_Counter>=Command_Rx_Max)
    {
			TDM_Rx_Counter = 0;      
			Command_Sort   = TDM_Rx_Data[0];
			
   
			if((Command_Sort==0x01)||(Command_Sort==0x06))    
			{
				AHRSComFlag    = 1;
				Data_Sort      = Command_Sort;
				Data_Com_Flag  = 1;                           //????
			}

		
			if((Command_Sort==0x41)||(Command_Sort==0x50)||(Command_Sort==0x51)||(Command_Sort==0x55))               //????
			{
				AHRSComFlag  = 1;
				Comm_Sort    = Command_Sort;
				Data_Com_Flag= 2;
			}
			
		 
		 if(Command_Sort==0x65) //??????
		 {
			if((TDM_Rx_Data[1]==15)&&(TDM_Rx_Data[14]==0xFF))
			{
			//--------------------------------------------------------
			 HVersionH  = (int16_t)(TDM_Rx_Data[2]*256+TDM_Rx_Data[3]);
			 HVersionL  = (int16_t)(TDM_Rx_Data[4]*256+TDM_Rx_Data[5]);
			//--------------------------------------------------------
			 SVersionH  = (int16_t)(TDM_Rx_Data[6]*256+TDM_Rx_Data[7]);
			 SVersionL  = (int16_t)(TDM_Rx_Data[8]*256+TDM_Rx_Data[9]);
			//--------------------------------------------------------
			 HVersionS  = (int16_t)(TDM_Rx_Data[10]*256+TDM_Rx_Data[11]);
			//--------------------------------------------------------
			
	

		 } 		 
		}
		 if(Command_Sort==0x60)                                //陀螺
		 {  
			if((TDM_Rx_Data[1]==15)&&(TDM_Rx_Data[14]==0xFF))
			{
			 YGyrBias  = (int16_t)(TDM_Rx_Data[2]*256)+(int16_t)(TDM_Rx_Data[3]);
			 XGyrBias  = (int16_t)(TDM_Rx_Data[4]*256)+(int16_t)(TDM_Rx_Data[5]);
			 ZGyrBias  = (int16_t)(TDM_Rx_Data[6]*256)+(int16_t)(TDM_Rx_Data[7]);

			 GyrBias_X = XGyrBias;
			 GyrBias_Y = YGyrBias;
			 GyrBias_Z = ZGyrBias;  
			 //--------------------------------------------------------
			 YGyrScale = (int16_t)(TDM_Rx_Data[8]*256)+ (int16_t)(TDM_Rx_Data[9]);
			 XGyrScale = (int16_t)(TDM_Rx_Data[10]*256)+(int16_t)(TDM_Rx_Data[11]);
			 ZGyrScale = (int16_t)(TDM_Rx_Data[12]*256)+(int16_t)(TDM_Rx_Data[13]);
			
			 GyrScale_X = XGyrScale; 
			 GyrScale_Y = YGyrScale;
			 GyrScale_Z = ZGyrScale;
			//--------------------------------------------------------

			}
		 }
		 else if(Command_Sort==0x61)                           //加速度计
		 {
			if((TDM_Rx_Data[1]==45)&&(TDM_Rx_Data[44]==0xFF))
			{
			 XAccBias = (int16_t)(TDM_Rx_Data[2]*256+TDM_Rx_Data[3]);
			 YAccBias = (int16_t)(TDM_Rx_Data[4]*256+TDM_Rx_Data[5]);
			 ZAccBias = (int16_t)(TDM_Rx_Data[6]*256+TDM_Rx_Data[7]);
			//--------------------------------------------------------
			 XAccScale = (int16_t)(TDM_Rx_Data[8]*256+TDM_Rx_Data[9]);
			 YAccScale = (int16_t)(TDM_Rx_Data[10]*256+TDM_Rx_Data[11]);
			 ZAccScale = (int16_t)(TDM_Rx_Data[12]*256+TDM_Rx_Data[13]); 
			//--------------------------------------------------------
			 XAccFScale = (int16_t)(TDM_Rx_Data[14]*256+TDM_Rx_Data[15]);
			 YAccFScale = (int16_t)(TDM_Rx_Data[16]*256+TDM_Rx_Data[17]);
			 ZAccFScale = (int16_t)(TDM_Rx_Data[18]*256+TDM_Rx_Data[19]); 



			}
    }
		 //--------------------------------------------			
  } 
}   


//--------------------------------------------
//void HandleUpdata(unsigned char UCdata)
//--------------------------------------------
void HandleUpdata(unsigned char UCdata)
{
	if(Up_Rx_Counter>=sizeof(Up_Rx_Data))
	{
   Up_Rx_Counter=sizeof(Up_Rx_Data)-1;
  }
	
  Up_Rx_Data[Up_Rx_Counter]=UCdata;
  Up_Rx_Counter++; 
	
	
  if(Up_Rx_Counter==1)
  {
   if(Up_Rx_Data[0]==0x55)   //update  /udebug
   {
    Up_Rx_Max =  6;    
   }  
   else
   {
    Up_Rx_Max =  1;
    Up_Rx_Counter=0;		 
   } 
  }
  else
  {
	 if(Up_Rx_Counter>=Up_Rx_Max)
	 {
     Up_Rx_Counter = 0;      
   
    if((Up_Rx_Data[0]==0x55)&&(Up_Rx_Data[1]==0x50)&&(Up_Rx_Data[2]==0x53)&&(Up_Rx_Data[3]==0x4F)&&(Up_Rx_Data[4]==0x46)&&(Up_Rx_Data[5]==0x54)) //UPSOFT
		{

			
			Update_One = 0x39;
   	  Update_Two = 0x93;
	    Update_Flash_W_Flag = 1;
			Up_On_Flag          = 2;
	  }	
	 }
	}

}

//------------------------------------------
//void HandlePassOn(unsigned char Comamnd)
//------------------------------------------
void HandlePassOn(unsigned char pComamnd)
{
 if(Pass_Rx_Counter>=sizeof(Pass_Rx_Data))   //保护
  {
   Pass_Rx_Counter=sizeof(Pass_Rx_Data)-1;
  }
 
  Pass_Rx_Data[Pass_Rx_Counter]=pComamnd;
  Pass_Rx_Counter++;

  if(Pass_Rx_Counter==1)
  {
   if(Pass_Rx_Data[0]==0x75)   //update  /udebug
   {
   Pass_Rx_Max =  6;    
   }  
   else
   {
   Pass_Rx_Max =  1;   
   Pass_Rx_Counter=0;		 
   } 
  }
  else
  {

   if(Pass_Rx_Counter>=Pass_Rx_Max)
   {
     Pass_Rx_Counter = 0;      
   
    if((Pass_Rx_Data[0]=='u')&&(Pass_Rx_Data[1]=='p')&&(Pass_Rx_Data[2]=='d')&&(Pass_Rx_Data[3]=='a')&&(Pass_Rx_Data[4]=='t')&&(Pass_Rx_Data[5]=='e'))
	  {
	   Up_On_Flag     = 1;
	  }
		else if((Pass_Rx_Data[0]=='u')&&(Pass_Rx_Data[1]=='d')&&(Pass_Rx_Data[2]=='e')&&(Pass_Rx_Data[3]=='b')&&(Pass_Rx_Data[4]=='u')&&(Pass_Rx_Data[5]=='g'))
		{
     Debug_Flag     = 1;
			
		 Buat_DGet_Flag     = 5;
		 USART_Configuration();	
    }
   }  
  }
}



//------------------------------------------
//void HandleKeyOn(unsigned char Comamnd)
//------------------------------------------
void HandleKeyOn(unsigned char KComamnd)
{
 	if(Key_Rx_Counter>=sizeof(Key_Rx_Data))
	{
   Key_Rx_Counter=sizeof(Key_Rx_Data)-1;
  }
	
	
  Key_Rx_Data[Key_Rx_Counter]=KComamnd;
  Key_Rx_Counter++;


  if(Key_Rx_Counter==1)
  {
   if(Key_Rx_Data[0]==0x6c)       //log gpggb/gpatt ontime 0.2  
   {
    Key_Rx_Max =  9;  
    }  
	 else if(Key_Rx_Data[0]==0x75)   //unlog gpggb/gpatt
   {
    Key_Rx_Max =  11;    
   }  
   else
   {
    Key_Rx_Max      = 1;   
    Key_Rx_Counter  = 0;		 
   } 
  }
  else
  {

   if(Key_Rx_Counter>=Key_Rx_Max)
   {
     Key_Rx_Counter = 0;      

		//关闭5HZ，打开1hz
		if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='h')&&(Key_Rx_Data[8]=='i')&&(Key_Rx_Data[9]=='g')&&(Key_Rx_Data[10]=='h'))
	  {
	   High_DGet_Flag     = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;
	  }
		//关闭ATT协议
		if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='a')&&(Key_Rx_Data[9]=='t')&&(Key_Rx_Data[10]=='t'))
	  {
	   ATT_DGet_Flag     = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;	
	  }
			//关闭GIRMC协议
		if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='i')&&(Key_Rx_Data[8]=='r')&&(Key_Rx_Data[9]=='m')&&(Key_Rx_Data[10]=='c'))
	  {
	   RMC_Back_Flag      = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;	
	  }
		//关闭INS协议
		else if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='i')&&(Key_Rx_Data[9]=='n')&&(Key_Rx_Data[10]=='s'))
	  {
	   INS_DGet_Flag      = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;				
		 g_GINavInfo.INSState = INS_ACTIVE;
	  }
		//关闭GSV协议
		else if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='g')&&(Key_Rx_Data[9]=='s')&&(Key_Rx_Data[10]=='v'))
	  {
		 GpsGsvClose();
	   GSV_DGet_Flag      = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;				
	  }
	  //关闭ZDA协议
		else if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='z')&&(Key_Rx_Data[9]=='d')&&(Key_Rx_Data[10]=='a'))
	  {
	   ZDA_DGet_Flag      = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;				
	  }
		//
		else if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='g')&&(Key_Rx_Data[9]=='p')&&(Key_Rx_Data[10]=='s'))
		{
		 SGPS_DGet_Flag     = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;				
    }
		//
		else if((Key_Rx_Data[0]=='u')&&(Key_Rx_Data[1]=='n')&&(Key_Rx_Data[2]=='l')&&(Key_Rx_Data[3]=='o')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='g')&&(Key_Rx_Data[9]=='b')&&(Key_Rx_Data[10]=='d'))
	  {
	   BD_DGet_Flag       = 0;
		 GpsGLOpen();		
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;				
	  }

		
	//-------------------------------------------------------------------------------------------------
		//关闭1HZ，打开5hz
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='h')&&(Key_Rx_Data[6]=='i')&&(Key_Rx_Data[7]=='g')&&(Key_Rx_Data[8]=='h'))
		{
     High_DGet_Flag     = 1;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
		//GPGGA
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='g')&&(Key_Rx_Data[8]=='a'))
		{
     GNGA_DGet_Flag     = 0;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }		
		//GNAA
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='n')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='g')&&(Key_Rx_Data[8]=='a'))
		{
     GNGA_DGet_Flag     = 1;
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;						
    }
		//---------------------------------------------------------------------------------
		//启动4800
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='4')&&(Key_Rx_Data[6]=='8')&&(Key_Rx_Data[7]=='0')&&(Key_Rx_Data[8]=='0'))
	  {
	   Buat_DGet_Flag     = 2;
		 High_DGet_Flag     = 0;
		 USART_Configuration();	
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
	  }
		//启动9600
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='9')&&(Key_Rx_Data[6]=='6')&&(Key_Rx_Data[7]=='0')&&(Key_Rx_Data[8]=='0'))
	  {
	   Buat_DGet_Flag     = 1;
		 High_DGet_Flag     = 0;
		 USART_Configuration();	
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
	  }
		//启动19200
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='1')&&(Key_Rx_Data[6]=='9')&&(Key_Rx_Data[7]=='2')&&(Key_Rx_Data[8]=='0'))
	  {
	   Buat_DGet_Flag     = 3;
		 High_DGet_Flag     = 0;
		 USART_Configuration();	
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
	  }
		//启动38400
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='3')&&(Key_Rx_Data[6]=='8')&&(Key_Rx_Data[7]=='4')&&(Key_Rx_Data[8]=='0'))
	  {
	   Buat_DGet_Flag     = 4;
		 High_DGet_Flag     = 0;
		 USART_Configuration();	
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
	  }
		//启动115200
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='1')&&(Key_Rx_Data[6]=='1')&&(Key_Rx_Data[7]=='5')&&(Key_Rx_Data[8]=='2'))
	  {
	   Buat_DGet_Flag     = 5;
		 USART_Configuration();	
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
	  }
		//---------------------------------------------------------------------------------	
		//打开ATT协议
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='a')&&(Key_Rx_Data[7]=='t')&&(Key_Rx_Data[8]=='t'))
		{
     ATT_DGet_Flag      = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		//---------------------------------------------------------------------------------	
		//打开GIRMC协议
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='i')&&(Key_Rx_Data[6]=='r')&&(Key_Rx_Data[7]=='m')&&(Key_Rx_Data[8]=='c'))
		{
     RMC_Back_Flag      = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		//-------------------------------------------------------------------
		//打开惯性导航
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='i')&&(Key_Rx_Data[7]=='n')&&(Key_Rx_Data[8]=='s'))
		{
     INS_DGet_Flag      = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		//-------------------------------------------------------------------
		//打开GSV协议
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='s')&&(Key_Rx_Data[8]=='v'))
		{
		 GpsGsvOpen();
     GSV_DGet_Flag      = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		//-------------------------------------------------------------------
	  //打开ZDA协议
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='z')&&(Key_Rx_Data[7]=='d')&&(Key_Rx_Data[8]=='a'))
		{
     ZDA_DGet_Flag      = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		
		//-------------------------------------------------------------------
	  //
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='s'))
		{
     SGPS_DGet_Flag     = 1;
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }		
		//-------------------------------------------------------------------
	  //打开BD协议
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='g')&&(Key_Rx_Data[5]=='p')&&(Key_Rx_Data[6]=='g')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='d'))
		{
     BD_DGet_Flag       = 1;
		 //GpsBDOpen();
			
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;					
    }
		

		
		//-------------------------------------------------------------------
	  //软件复位
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='r')&&(Key_Rx_Data[5]=='e')&&(Key_Rx_Data[6]=='s')&&(Key_Rx_Data[7]=='e')&&(Key_Rx_Data[8]=='t'))
		{
//     SoftReset();			
    }
		
		//-------------------------------------------------------------------
		//  坐标系修改
		//-------------------------------------------------------------------
	  
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='1'))
		{
     IMU_Kind           = 1;
			
		 Data_Flash_W_Flag   = 1;
		 Up_On_Flag          = 2;					
    }  
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='2'))
		{
     IMU_Kind           = 2;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='3'))
		{
     IMU_Kind           = 3;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='4'))
		{
     IMU_Kind          = 4;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
		}
    else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='5'))
		{
     IMU_Kind          = 5;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
	
		//-------------------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='6'))
		{
     IMU_Kind           = 6;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='7'))
		{
     IMU_Kind           = 7;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='z')&&(Key_Rx_Data[5]=='u')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='b')&&(Key_Rx_Data[8]=='8'))
		{
     IMU_Kind           = 8;
						
		 Data_Flash_W_Flag  = 1;
		 Up_On_Flag         = 2;		
    }

		
	//-----------------log clear-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='c')&&(Key_Rx_Data[5]=='l')&&(Key_Rx_Data[6]=='e')&&(Key_Rx_Data[7]=='a')&&(Key_Rx_Data[8]=='r'))
		{
		 ANG_FGet_Flag      = 0;		
     ANG_DGet_Flag      = 0;	
		 ANG_Lock_Flag      = 0;			
		 MisAngleData       = 0;	

		 RSE_FGet_Flag      = 0;	
		 Lat_Back           = 0.0;    //维度
     Lon_Back           = 0.0;    //经度
     Head_Back          = 0.0;    //方向
			
		 MAngle_Flash_W_Flag  = 1;			
		 Up_On_Flag           = 2;		
		}
		
		//-----------------log alock-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='a')&&(Key_Rx_Data[5]=='l')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='c')&&(Key_Rx_Data[8]=='k'))
		{
		 ANG_Lock_Flag        = 1;			
		 MAngle_Flash_W_Flag  = 1;			
		 Up_On_Flag           = 2;		   	 	
		}		
	  //-----------------log aauto-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='a')&&(Key_Rx_Data[5]=='a')&&(Key_Rx_Data[6]=='t')&&(Key_Rx_Data[7]=='u')&&(Key_Rx_Data[8]=='o'))
		{
		 ANG_Lock_Flag      = 0;	
			
		 ANG_FGet_Flag      = 0;		
     ANG_DGet_Flag      = 0;			
		 MisAngleData       = 0;	

		 RSE_FGet_Flag      = 0;	
		 Lat_Back           = 0.0;    //维度
     Lon_Back           = 0.0;    //经度
     Head_Back          = 0.0;    //方向
			
		 MAngle_Flash_W_Flag  = 1;			
		 Up_On_Flag           = 2;			
		}
		
		//-----------------log fixed-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='f')&&(Key_Rx_Data[5]=='i')&&(Key_Rx_Data[6]=='x')&&(Key_Rx_Data[7]=='e')&&(Key_Rx_Data[8]=='d'))
		{
			 if((IMU_Kind>0)&&(IMU_Kind<6))
			 {
        ANG_Kind_Flag       = 1;
			
		    MAngle_Flash_W_Flag = 1;			
		    Up_On_Flag          = 2;		
			 }
		}
		
	  //-----------------log insgo-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='i')&&(Key_Rx_Data[5]=='n')&&(Key_Rx_Data[6]=='s')&&(Key_Rx_Data[7]=='g')&&(Key_Rx_Data[8]=='o'))
		{
   	 Ins_Go_Flag=1;	
		 g_GINavInfo.Ini4_Flag=1;
			
		 g_GINavInfo.Acc_D_Num = 10;
     g_GINavInfo.Acc_D_Angle=12;
		}
		
		 //-----------------log cloli-----------------------------------------------------------
		else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='c')&&(Key_Rx_Data[5]=='l')&&(Key_Rx_Data[6]=='o')&&(Key_Rx_Data[7]=='l')&&(Key_Rx_Data[8]=='i'))
		{
	 	 g_GINavInfo.OliData=0;
		}	
    //-----------------log lowpo-----------------------------------------------------------		
    else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='l')&&(Key_Rx_Data[5]=='o')&&(Key_Rx_Data[6]=='w')&&(Key_Rx_Data[7]=='p')&&(Key_Rx_Data[8]=='o'))
		{
	    Lowpow_Flag       = 1;
		}		
		//-----------------log sleep-----------------------------------------------------------
		    else if((Key_Rx_Data[0]=='l')&&(Key_Rx_Data[1]=='o')&&(Key_Rx_Data[2]=='g')&&(Key_Rx_Data[4]=='s')&&(Key_Rx_Data[5]=='l')&&(Key_Rx_Data[6]=='e')&&(Key_Rx_Data[7]=='e')&&(Key_Rx_Data[8]=='p'))
		{
	    Lowpow_Sleep      = 1;
		}	
		
   } 
  }
}









void SerialPutChar(uint8_t c)
{
	if(Uart_Kind==1)
	{
   USART_SendData(USART2, c);
   while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
   {
   }
  }
	else
	{
   USART_SendData(USART1, c);
   while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
   {
   }
  }
}



void Serial_PutString(uint8_t *s)
{
  while (*s != '\0')
  {
    SerialPutChar(*s);
    s++;
  }
}


//---------------------------------
//void SendText(void)
//---------------------------------
void SendText(void)
{
static uint16_t i;

if(Uart_Kind==1)
{
 for(i=0;i<Command_Tx_Max;i++)
 {
  USART_SendData(USART2, TDM_TX_Data[i]);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);	 
 }
}
 else
 {
 for(i=0;i<Command_Tx_Max;i++)
 {
  USART_SendData(USART1, TDM_TX_Data[i]);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);	 
 }
}

 Command_Tx_Max=0;
}


//---------------------------------
//void SendVersion(void)
//---------------------------------
void SendVersion(void)
{
	  uint8_t i;
	
	
//-------------------------------
 if(User_Kind==7)
	{
		SerialPutString("\r\n");
		SerialPutString("==================================================================\r\n");
	  SerialPutString("===================   SKM-4DU  System  ============================\r\n");
		SerialPutString("===================  www.skylab.com.cn ============================\r\n");
	  SerialPutString("==================================================================\r\n");
		SerialPutString("Startup Application...  \r\n\n");
	
	}
  else
	{		
		SerialPutString("\r\n");
		SerialPutString("==================================================================\r\n");
	  SerialPutString("===================   GI-200 System   ============================\r\n");
		SerialPutString("===================   www.hmnav.com   ============================\r\n");	
	  SerialPutString("==================================================================\r\n");
		SerialPutString("Startup Application...  \r\n\n");
	}
	
			
		//-------------------------------------------
		//software version
		//-------------------------------------------
	  SerialPutString("Software version:   ");	
		

		ProcessVTOA(SVersionH);
		for(i=0;i<4;i++)
		{
		 TDM_TX_Data[i] = GGA_Data[i];	 
		}
		
		ProcessVTOA(SVersionL);
		for(i=0;i<4;i++)
		{
		 TDM_TX_Data[4+i] = GGA_Data[i];	 
		}
		
		TDM_TX_Data[8]=0x0D;
		TDM_TX_Data[9]=0x0A;
		
		Command_Tx_Max=10;
		
		SendText();
		
		//-------------------------------------------
		//hardware serial
		//-------------------------------------------
		
		SerialPutString("Hardware serial:    ");
		
		ProcessVTOA(HVersionH);
		for(i=0;i<4;i++)
		{
		 TDM_TX_Data[i] = GGA_Data[i];	 
		}
		
		ProcessVTOA(HVersionL);
		for(i=0;i<4;i++)
		{
		 TDM_TX_Data[4+i] = GGA_Data[i];	 
		}
		
		TDM_TX_Data[8]='-';
		TDM_TX_Data[9]='-';
		
		
		ProcessVTOA(HVersionS);
		for(i=0;i<4;i++)
		{
		 TDM_TX_Data[10+i] = GGA_Data[i];	 
		}
		
		TDM_TX_Data[14]=0x0D;
		TDM_TX_Data[15]=0x0A;
		
		Command_Tx_Max=16;
		SendText();
		
		//-------------------------------------------
		//software version
		//-------------------------------------------
		SerialPutString("\r\n");
		
	  SerialPutString("Configuration:\r\n");
		
		if(Debug_Flag)
		{
			SerialPutString("  COM1   : 115200 bps\r\n");
			SerialPutString("  COM2   : 115200 bps\r\n"); 
			SerialPutString("  Output : GGA RMC ATT\r\n");
		}
		else
		{
		if(Buat_DGet_Flag==1)  
		{
			SerialPutString("  COM1   : 9600 bps\r\n");
			SerialPutString("  COM2   : 9600 bps\r\n"); 
			SerialPutString("  Output : GGA RMC ATT\r\n");
		}
		else if(Buat_DGet_Flag==2) 
		{
			SerialPutString("  COM1   : 4800 bps\r\n");
			SerialPutString("  COM2   : 4800 bps\r\n"); 
			SerialPutString("  Output : GGA RMC ATT\r\n");
		}
		else if(Buat_DGet_Flag==3) 
		{
			SerialPutString("  COM1   : 19200 bps\r\n");
			SerialPutString("  COM2   : 19200 bps\r\n"); 
			SerialPutString("  Output : GGA RMC ATT\r\n");
		}
		else if(Buat_DGet_Flag==4) 
		{
			SerialPutString("  COM1   : 38400 bps\r\n");
			SerialPutString("  COM2   : 38400 bps\r\n"); 
			SerialPutString("  Output : GGA RMC ATT\r\n");
		}
		else
		{
			SerialPutString("  COM1   : 115200 bps\r\n");
			SerialPutString("  COM2   : 115200 bps\r\n"); 			
		}
		if(High_DGet_Flag)
		{
		 SerialPutString("  Rate   : 5HZ\r\n");	
		}
		else
		{
		 SerialPutString("  Rate   : 1HZ\r\n");	
		}
 }


}



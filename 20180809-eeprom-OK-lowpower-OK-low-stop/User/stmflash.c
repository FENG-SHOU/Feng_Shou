#include "stm32f4xx.h"                  // Device header
#include "stmflash.h"
#include "commdata.h"
#include "hw_config.h"
#include "delay.h"
#include "GetAcc.h"
#include "GetGyr.h"
#include "GetNass.h"
#include <string.h>


//**********************************************
//                 Flash������
//**********************************************

//---------------------------------------------------
//
//---------------------------------------------------

void Update_Flash_INI(void)
{
  Flash_Wdata[0] = Update_One;   
  Flash_Wdata[1] = Update_Two; 
}

void SoftReset(void)
{	
	__set_FAULTMASK(1);// ??????	
	NVIC_SystemReset();// ??
}



void Update_Flash_Save(void)
{
 if(Update_Flash_W_Flag&&Flash_Wirte_Flag) 
 {
  Update_Flash_W_Flag = 0;
  Flash_Wirte_Flag    = 0;

  Flash_Wing_Flag   = 1;
	
	NVIC_ConfigurationDis();		
	 
  Update_Flash_INI(); 
	
	__disable_irq();	 
	STMFLASH_Write(ADDR_FLASH_SECTOR_2 ,(u32*)Flash_Wdata,UpData_Buff_Size);
	__enable_irq();
	 
  Flash_Wing_Flag   = 0;  

	NVIC_Configuration();
	 
	Up_On_Flag        = 0;
	 
  SoftReset();
 }
}


//-------------------------------------------------------------
//
//-------------------------------------------------------------


void MisAngle_Flash_INI(void)
{
	uint8_t i;
	char sBuf[8];
  char* temp;
  double Dtemp;

	Flash_Wdata[0] = ANG_FGet_Flag;
	Flash_Wdata[1] = ANG_Kind_Flag;
  Flash_Wdata[2] = ANG_Lock_Flag;	
	
	Flash_Wdata[3]  = MisAngleData>>0x08;
  Flash_Wdata[4]  = MisAngleData;
	
	Flash_Wdata[5] = Ini0_KindNew;	
	
	Flash_Wdata[6] = Ini0_RollNew>>0x08;
  Flash_Wdata[7] = Ini0_RollNew;
	
	Flash_Wdata[8] = Ini0_PitchNew>>0x08;
  Flash_Wdata[9] = Ini0_PitchNew;
	
	//------------------------------------------------------
	//10-17
	//------------------------------------------------------
   memset(sBuf,0,sizeof(sBuf));
	 Dtemp=GINavResult.Position.Lat;
   temp=(char*)(&Dtemp);
   sBuf[0] = temp[0] ;
   sBuf[1] = temp[1];
   sBuf[2] = temp[2];
   sBuf[3] = temp[3]; 
   sBuf[4] = temp[4] ;
   sBuf[5] = temp[5];
   sBuf[6] = temp[6];
   sBuf[7] = temp[7]; 	
	 
	 for(i=0;i<8;i++)
	 {
	  Flash_Wdata[10+i]=sBuf[i];
	 }	
	//------------------------------------------------------
	//18-25
	//------------------------------------------------------ 
	 memset(sBuf,0,sizeof(sBuf));
	 Dtemp=GINavResult.Position.Lon;
   temp=(char*)(&Dtemp);
   sBuf[0] = temp[0] ;
   sBuf[1] = temp[1];
   sBuf[2] = temp[2];
   sBuf[3] = temp[3]; 
   sBuf[4] = temp[4] ;
   sBuf[5] = temp[5];
   sBuf[6] = temp[6];
   sBuf[7] = temp[7]; 	
	 
	 for(i=0;i<8;i++)
	 {
	  Flash_Wdata[18+i]=sBuf[i];
	 }
	
	//-----------------------------------------------
	 Pitch_Back=GINavResult.Attitude.Pitch*RAD2DEG;	
	 Flash_Wdata[26]  = Pitch_Back>>0x08;
   Flash_Wdata[27]  = Pitch_Back;
	
	 Roll_Back=GINavResult.Attitude.Roll*RAD2DEG;	
	 Flash_Wdata[28]  = Roll_Back>>0x08;
   Flash_Wdata[29]  = Roll_Back;
	
	 Head_Back=GINavResult.Attitude.Heading*RAD2DEG;	 
	 Flash_Wdata[30]  = Head_Back>>0x08;
   Flash_Wdata[31]  = Head_Back;
	//-----------------------------------------------
	 
	 Flash_Wdata[32]=RSE_FGet_Flag;
	 
	 Flash_Wdata[33]=NorthSouthSelect;
	 
	 Flash_Wdata[34]=WestEastSelect; 
	 
	 
	 
}

void MisAngle_Flash_Decode(void)
{
  STMFLASH_Read(ADDR_FLASH_SECTOR_3,(u32*)Flash_Rdata,MAgnle_Buff_Size);   //??flash????

	ANG_DGet_Flag  =  Flash_Rdata[0];	
	ANG_Kind_Flag  =  Flash_Rdata[1];	
	ANG_Lock_Flag  =  Flash_Rdata[2];	
	
	MisAngleData   =  Flash_Rdata[3]*256+Flash_Rdata[4];
		
  Ini0_KindBack  =  Flash_Rdata[5];                     //��װ��ʽ
  Ini0_RollBack  =  Flash_Rdata[6]*256+Flash_Rdata[7];  //��װ��
  Ini0_PitchBack =  Flash_Rdata[8]*256+Flash_Rdata[9];  //��װ��
	
	
	//--------------------------------------------------------	
  Lat_Back  = Buftodouble(Flash_Rdata,10);              //ά��10-17
  Lon_Back  = Buftodouble(Flash_Rdata,18);              //����18-25
  
	Pitch_Back = Flash_Rdata[26]*256+Flash_Rdata[27];     //��̬��
	Roll_Back  = Flash_Rdata[28]*256+Flash_Rdata[29];     //��̬��
	Head_Back  = Flash_Rdata[30]*256+Flash_Rdata[31];	    //��̬��
	
	RSE_DGet_Flag  =  Flash_Rdata[32];
  NorthSouthBack =  Flash_Rdata[33];                   //
  WestEastBack   =  Flash_Rdata[34];                   //
	
	//----------------------------------------
	//�رճ����Ƶ����⣡
	//----------------------------------------
 	RSE_DGet_Flag  = 0;                 
  //----------------------------------------
	
	if((Lat_Back!=Lat_Back)||(Lon_Back!=Lon_Back))
	{
	 RSE_DGet_Flag=0;
	}
	
  if((RSE_DGet_Flag!=0)&&(RSE_DGet_Flag!=1))
	{ 	
   RSE_DGet_Flag= 0;                                   //0����  1����
	}
	//--------------------------------------------------------	
	//
	//--------------------------------------------------------	 
	if((ANG_Kind_Flag!=0)&&(ANG_Kind_Flag!=1))
	{ 	
   ANG_Kind_Flag= 0;                                   //0������Ӧ(Ĭ��)...1���̶�
	}

	//--------------------------------------------------------		 
	if((ANG_DGet_Flag!=0)&&(ANG_DGet_Flag!=1))
	{ 	
		ANG_DGet_Flag= 0;                                    //0��û�д洢��.��Ĭ�ϣ�..  1:�洢��
	}
	
	//--------------------------------------------------------	
  if((ANG_Lock_Flag!=0)&&(ANG_Lock_Flag!=1))
	{ 	
   ANG_Lock_Flag =0;                                    //0�� û��������Ĭ�ϣ�       1������
	}
	
	//--------------------------------------------------------		
	if((Ini0_KindBack>=1)&&(Ini0_KindBack<=6))
	{
	
	}
  else
	{
		MisAngleData  = 0;
	  ANG_DGet_Flag = 0;
		ANG_Lock_Flag = 0;	
		RSE_DGet_Flag = 0;                                   
	}
	
	//--------------------------------------------------------	
  if((NorthSouthBack!=1)&&(NorthSouthBack!=2))
	{ 	
		NorthSouthBack =1;                                   //1:N  2:S
	}
		
 //--------------------------------------------------------	
  if((WestEastBack!=1)&&(WestEastBack!=2))
	{ 	
		WestEastBack =1;                                    //1:E  2:W
	}
	
  //--------------------------------------------------------	
	if((MisAngleData>360)||(MisAngleData<-360))           //����Ƕ���Ч��������
	{
	  MisAngleData  = 0;
	  ANG_DGet_Flag = 0;
		ANG_Lock_Flag = 0;
	}

	//--------------------------------------------------------	
	if((Flash_Rdata[3]==0xFF)&&(Flash_Rdata[4]==0xFF))    //���û�����ù���������
	{
	  MisAngleData  = 0;
	  ANG_DGet_Flag = 0;
		ANG_Lock_Flag = 0;
	}
	
	//----------------------------------------------
	// RSE_SGet_Flag��¼�����������
	//----------------------------------------------
	if(RSE_DGet_Flag==1)
	{
	 RSE_SGet_Flag=1;
	}
	else
	{
	 RSE_SGet_Flag=0;
	}
	//----------------------------------------------

}

//---------------------------------------------
//void MisAngle_Flash_Save(void)
//---------------------------------------------
void MisAngle_Flash_Save(void)
{
	int i;
	uint8_t write_flag;
 if(MAngle_Flash_W_Flag&&Flash_Wirte_Flag) 
 {
  Update_Flash_W_Flag = 0;
  Flash_Wirte_Flag    = 0;

  Flash_Wing_Flag   = 1;
	
	NVIC_ConfigurationDis();		
	 
  MisAngle_Flash_INI(); 
	 
	STMFLASH_Read(ADDR_FLASH_SECTOR_3,(u32*)Flash_Rdata,MAgnle_Buff_Size);   //
	 
	write_flag = 0;
		
	 for (i = 0; i < MAgnle_Buff_Size; i++)
	 {
		if (Flash_Rdata[i] != Flash_Wdata[i])
		{
			write_flag = 1;
			break;
		}
	 }		 
	 
	 if(write_flag)
	 {
		 __disable_irq();
		 STMFLASH_Write(ADDR_FLASH_SECTOR_3 ,(u32*)Flash_Wdata,MAgnle_Buff_Size);
		 __enable_irq();	 
	 }
		 
		
  Flash_Wing_Flag   = 0;  

	NVIC_Configuration();
	 
	Up_On_Flag        = 0;
 }
}


//---------------------------------------------------
//
//---------------------------------------------------



void Para_Flash_INI(void)
{
  //----------------------------------------------

  //----------------------------
  Flash_Wdata[0]=HVersionH>>0x08;
  Flash_Wdata[1]=HVersionH;

  Flash_Wdata[2]=HVersionL>>0x08;
  Flash_Wdata[3]=HVersionL;
//----------------------------
  Flash_Wdata[4]=SVersionH>>0x08;
  Flash_Wdata[5]=SVersionH;

  Flash_Wdata[6]=SVersionL>>0x08;
  Flash_Wdata[7]=SVersionL;
  //----------------------------
  Flash_Wdata[8]=HVersionS>>0x08;
  Flash_Wdata[9]=HVersionS;
  //----------------------------  
	
	Flash_Wdata[10] = INS_DGet_Flag; 
	Flash_Wdata[11] = High_DGet_Flag; 
	Flash_Wdata[12] = GSV_DGet_Flag; 
	Flash_Wdata[13] = ATT_DGet_Flag; 
	Flash_Wdata[14] = Buat_DGet_Flag; 
	Flash_Wdata[15] = GNGA_DGet_Flag; 
	Flash_Wdata[16] = ZDA_DGet_Flag;
	Flash_Wdata[17] = BD_DGet_Flag;
	
	Flash_Wdata[18] = IMU_Kind;
  Flash_Wdata[19]	= RMC_Back_Flag;

}

 
 void Para_Flash_Decode(void)
{
 
 //--------------------------------------------------------
  HVersionH  = Flash_Rdata[0]*256+ Flash_Rdata[1];    //2015    hardware version
  HVersionL  = Flash_Rdata[2]*256+ Flash_Rdata[3];   //1201    hardware version
  //--------------------------------------------------------
  SVersionH  =  Flash_Rdata[4]*256+Flash_Rdata[5];    //2015    soft version
  SVersionL  =  Flash_Rdata[6]*256+Flash_Rdata[7];    //1221    soft version
  //--------------------------------------------------------
  HVersionS  =  Flash_Rdata[8]*256+Flash_Rdata[9];     //0001    hardware version
		
	INS_DGet_Flag  = 	Flash_Rdata[10];
  High_DGet_Flag = 	Flash_Rdata[11];
	GSV_DGet_Flag  = 	Flash_Rdata[12];
	ATT_DGet_Flag  = 	Flash_Rdata[13];
	Buat_DGet_Flag =  Flash_Rdata[14];
	GNGA_DGet_Flag =  Flash_Rdata[15];
	ZDA_DGet_Flag  =  Flash_Rdata[16];
	BD_DGet_Flag   =  Flash_Rdata[17];
	IMU_Kind       =  Flash_Rdata[18];
	RMC_Back_Flag  =  Flash_Rdata[19];
	
	

	
//------------------------------------------------------
  Para_Data_Protection();
//------------------------------------------------------
}


void Para_Data_Protection(void)
{
  
   Data_Sort= 0x06;	 
	 //--------------------------------
	 //   ����汾
	 //--------------------------------
	 SVersionH  =  2018;
	 SVersionL  =   808; 
	 
	 //--------------------------------
	 //  Ӳ���汾
	 //--------------------------------	 
	
	 HVersionH   = 2018;  
	 HVersionL   = 501;
	 HVersionS   = 1;
 		 
	 //--------------------------------
	 //
	 //--------------------------------
   INS_DGet_Flag=1;     //�򿪹���   
	
	 //---------------------------------------------
	 if((High_DGet_Flag!=0)&&(High_DGet_Flag!=1))
	 {
		 if((User_Kind==4)||(User_Kind==8))
		 {
		  High_DGet_Flag=1;    //5HZ
		 }
		 else
		 {
	    High_DGet_Flag=0;    //1HZ
		 }
	 }
	 
		//--------------------------------------------- 
	 if((RMC_Back_Flag!=0)&&(RMC_Back_Flag!=1))
	 { 	
   	RMC_Back_Flag= 0;
	 }
	//---------------------------------------------
	 if((GSV_DGet_Flag!=0)&&(GSV_DGet_Flag!=1))
	 {
		 if(User_Kind==9)
	   {
		  GSV_DGet_Flag=1;		  //Ĭ�ϴ�
	   }
	   else
	   {
	    GSV_DGet_Flag=0;     //GSV	
	   }
   }
	 

	//---------------------------------------------	
    ATT_DGet_Flag=1;     //ATT
	//---------------------------------------------
	 if((Buat_DGet_Flag<0)||(Buat_DGet_Flag>5))     //�ǳ�Σ�գ�����������ע������ط���Data_Ini���Ⱥ󣡣�
	 {	
		Buat_DGet_Flag=5;		  //115200	
	 }
	 
	 if((User_Kind==2)||(User_Kind==3)||(User_Kind==4)||(User_Kind==6))
	 {
	 	 Buat_DGet_Flag=5;		  //115200	 
	 }
	 
	 //---------------------------------------------
	 if((User_Kind==5)||(User_Kind==9))
	 {
		 Buat_DGet_Flag=1;		  //9600
	 }
	 //---------------------------------------------
	 if((Ttx_data[1]!=Ddx_data[1])||(Ttx_data[0]!=Ddx_data[0]))  //
	 {
	  INS_DGet_Flag=0;
	 }
	//---------------------------------------------
   GNGA_DGet_Flag=1;
 	//---------------------------------------------
	
   ZDA_DGet_Flag=0;	 
 	//---------------------------------------------
	
	 if((BD_DGet_Flag!=0)&&(BD_DGet_Flag!=1))
	 {
	  BD_DGet_Flag =0;	   
	 }   
	  
 	//---------------------------------------------

	//--------------------------------------------	
	if((SGPS_DGet_Flag!=0)&&(SGPS_DGet_Flag!=1))
	{
		 SGPS_DGet_Flag =0;	 
	}
	//--------------------------------------------
	//Ĭ��Ϊ1..
	//--------------------------------------------
	 if((IMU_Kind<1)||(IMU_Kind>8))
	 {
	  IMU_Kind=1;
	 }
	//--------------------------------------------
	//�����װ��������Ӧ��װ..��̶�Ϊ1...
	//--------------------------------------------
	 if(ANG_Lock_Flag==0) 	
   IMU_Kind=1;
	 

	 
}
 
 //****************************************************

void Data_Flash_Decode(void)
{
 STMFLASH_Read(ADDR_FLASH_SECTOR_1,(u32*)Flash_Rdata,Data_Buff_Size);   //??flash????

 Para_Flash_Decode();
	
}


void Data_Flash_Save(void)
{
	int i;
	uint8_t write_flag = 0;
	
  if(Data_Flash_W_Flag&&Flash_Wirte_Flag) 
  {
		 Data_Flash_W_Flag = 0;
		
		 Flash_Wirte_Flag = 0;
	 	 Flash_Wing_Flag   = 1;	
		
		 Para_Flash_INI(); 
		
     STMFLASH_Read(ADDR_FLASH_SECTOR_1,(u32*)Flash_Rdata,Data_Buff_Size); 
		
		 
		 write_flag = 0;
		
		 for (i = 10; i < Data_Buff_Size; i++)
		 {
				if (Flash_Rdata[i] != Flash_Wdata[i])
				{
						write_flag = 1;
						break;
				}
		 }
		 
		 
		 if(write_flag)
		 {
			 __disable_irq();
		   STMFLASH_Write(ADDR_FLASH_SECTOR_1,(u32*)Flash_Wdata,Data_Buff_Size);
       __enable_irq();	 
		 }

		 Up_On_Flag = 0;
		 write_flag = 0;
		 Flash_Wirte_Flag = 0;
		 Flash_Wing_Flag   = 0;
	}
	
	//if(BD_DGet_Flag)
		
}



double Buftodouble(uint8_t *Buf,uint8_t counter)
{
	double a=0.0; 
	unsigned char * b = (unsigned char*)&a;
	uint8_t i = 0;

	for(i=0;i<8;i++,counter++)
	{
		b[i] = Buf[counter];
	}
	return a;
}


//****************************************************
//---------------------------------------------------
//
//---------------------------------------------------
 
 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	
	if(WriteAddr<ADDR_FLASH_SECTOR_1)return;//�Ƿ���ַ
	if(WriteAddr>ADDR_FLASH_SECTOR_4)return;//�Ƿ���ַ
	
	FLASH_Unlock();									//���� 
	
  FLASH_DataCacheCmd(DISABLE);    //FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				        //д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			      //ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)	       	//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}


















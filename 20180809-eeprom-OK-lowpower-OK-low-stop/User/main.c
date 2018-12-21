#include "stm32f4xx.h"
#include "24cxx.h" 
#include "stmflash.h"

#include "delay.h"
#include "hw_config.h"
#include "EncryptID.h"


#include "GetAcc.h"
#include "GetGyr.h"
#include "GetImu.h"

#include "GetGPS.h"
#include "SendGPS.h"

#include "CommData.h"

#include "datatypes.h"
#include "GINavMain.h"

#include "iwdg.h"

#include "EEPROM.h" 
//-----------------------------------------
//   GetAcc.h		
//-----------------------------------------
uint32_t   USART2_BodRate;

//-----------------------------------------
//   GetAcc.h		
//-----------------------------------------

//uint8_t  BMI055A_Rx[6];	   //�交�漲霈∪�憪�?
uint8_t  BMI160A_Rx[6];
int16_t	 ACC_X;			       //�漲霈—頧湔�?
int16_t  ACC_Y;			       //�漲霈︳頧湔�?
int16_t  ACC_Z;			       //�漲霈╴頧湔�?

//-----------------------------------------
int16_t XAccBias;
int16_t XAccScale;
int16_t XAccFScale;
//-----------------------------------
int16_t YAccBias;
int16_t YAccScale;
int16_t YAccFScale;
//-----------------------------------
int16_t ZAccBias;
int16_t ZAccScale;
int16_t ZAccFScale;
//-----------------------------------

float   floPitchAcc;
float   floRollAcc;
float   floAliAcc;

float   floPitchAngle;
float   floRollAngle;

int16_t  intPitchAcc;		   //
int16_t  intRollAcc;			 //
int16_t  intAliAcc;			   //

int16_t  intPitchAngle;	   //�箔��漲霈∠�靽臭趕閫?
int16_t  intRollAngle;	   //�箔��漲霈∠�璅芣�閫?
int16_t  intYawAngle;      //�箔�閫漲���

//-----------------------------------------
//    GetGyr.h
//-----------------------------------------
//uint8_t BMI055G_Rx[6];	  //
uint8_t BMI160G_Rx[6];


int16_t  Gyr_X;			      //��箔貌X頧湔�?
int16_t  Gyr_Y;			      //��箔貌Y頧湔�?
int16_t  Gyr_Z;			      //��箔貌Z頧湔�?

int16_t  GyrBias_X;       // ��箔貌�膳隞啗蓬�蔭摰��   
int16_t  GyrBias_Y;       // ��箔貌�膳隞啗蓬�蔭摰��   
int16_t  GyrBias_Z;       // ��箔貌�膳隞啗蓬�蔭摰��

int16_t  GyrScale_X;      // ��箔貌�頧湔�蝘啣�摮? store
int16_t  GyrScale_Y;      // ��箔貌�頧湔�蝘啣�摮? store
int16_t  GyrScale_Z;      // ��箔貌�頧湔�蝘啣�摮? store 

int16_t  XGyrBias;        // ��箔貌�膳隞啗蓬�蔭   store
int16_t  YGyrBias;        // ��箔貌�膳隞啗蓬�蔭   store
int16_t  ZGyrBias;        // ��箔貌�膳隞啗蓬�蔭   store

int16_t  XGyrScale;       // ��箔貌�頧湔�蝘啣�摮? store
int16_t  YGyrScale;       // ��箔貌�頧湔�蝘啣�摮? store
int16_t  ZGyrScale;       // ��箔貌�頧湔�蝘啣�摮? store 

float  floPitchGyr;		  //???璉
float  floRollGyr;		  //???璉
float  floYawGyr;		    //???璉

float   floYawAngle;

int16_t intPitchGyr;	    // 
int16_t intRollGyr;		    // 
int16_t intYawGyr;		    // 

uint8_t IMU_Kind;
uint8_t DIni_Flag;


uint8_t GpsKind;
uint8_t ProductKind;


//--------------------------------------------------
//     GetIMU.h
//--------------------------------------------------
//--------------------------------------
int16_t   IMU_SData[51][8];     //��摮�啁�.....
uint16_t  DAT_SData[21][4];     //

uint8_t   m_IMUData[300];       //
uint8_t   m_IMUDataB[300];      //

uint8_t   IMUTC_Data[20];       //UTC�園....
uint8_t   IMU_DATAT[60];        //�其�摮IMU�
//--------------------------------------------------

uint8_t   FrameFirst;

uint8_t   BaseTime;        //
uint8_t   DetaTime;        //

uint8_t   SaveTime;        //���啁�蝝Ｗ�....
uint8_t   SendTime;        //SendTime�aveTime銋�閬�颲��隞乩�摰釣��閬�靘蹂耨�?...
uint8_t   PackTime;        //PackTime�aveTime銋�閬�颲��隞乩�摰釣��閬�靘蹂耨�?...

uint8_t   IMU_Send_Flag;

uint16_t  IMU_DMA_Num;

uint16_t  IMDMStime;

uint8_t   IMDHour;						//	 UTC�園嚗
uint8_t   IMDMinute;					//	 �?
uint8_t   IMDSecond;					//	 蝘?
uint8_t   IMDMSecond;        //   瘥怎�

uint8_t   IMU_Flag;          //
uint8_t   Ins_Go_Flag;
//--------------------------------------
uint8_t    T1times;
uint16_t   T2times;
uint16_t   T3times;

uint8_t    Change_Data[50];
uint8_t    IMU_Num;           //霂餃�IMU�唳�活�?
//-------------------------------------------
//   GetGps.h
//-------------------------------------------
uint8_t   m_GSPData[NP_ALL_DATA_LEN];       // NMEA data     900  GGA+RMC+GSA+GSV+GST
uint8_t   m_GSPDataB[NP_ALL_DATA_LEN];      // NMEA data     900  GSA+GSV+GST

uint8_t   m_GPSpData[NP_MAX_DATA_LEN];       // NMEA data     NMEA�唳
uint8_t   m_RMCpData[NP_MAX_DATA_LEN];      // NMEA data     NMEA�唳

uint8_t     GGA_Get_Flag;        //Gps�瑕���
uint8_t     RMC_Get_Flag;        //Gps�瑕���
uint8_t     VTG_Get_Flag;        //Gps�瑕���
uint8_t     GST_Get_Flag;        //Gps�瑕���
uint8_t   	GSA_Get_Flag;	       //Gps�瑕���
uint8_t  	  GSV_Get_Flag;        //Gps�瑕���


uint8_t GGA_Config_flag;
uint8_t GST_Config_flag;
uint8_t GSA_Config_flag;
uint8_t GSV_Config_flag;
uint8_t GLL_Config_flag;
uint8_t VTG_Config_flag;
uint8_t ZDA_Config_flag;
uint8_t RMC_Config_flag;
uint8_t BD1_Config_flag;
uint8_t BD2_Config_flag;
uint8_t GL_Config_flag;
uint8_t RATE_Config_flag;
uint8_t PRT_Config_flag;
uint8_t timer_start_flag;
uint16_t ttime;

GPGGA_DATA  GPGGAData;				    //蝏�雿�摮�葵�悅銝剔��捆
GPRMC_DATA  GPRMCData;
GPVTG_DATA  GPVTGData;
GPGST_DATA  GPGSTData;
GPGSA_DATA  GPGSAData;
GPGSV_DATA  GPGSVData;
GPZDA_DATA  GPZDAData;

uint8_t     NorthSouthSelect;
uint8_t     WestEastSelect;

uint8_t    GetAgeDataFlag;   //
uint16_t   GetAgeData;       //
uint8_t    InPutDiffNum;       //
uint8_t    Diff_First_Flag;
uint8_t    Gps_Fren_Kind;
uint8_t    Rmc_Vtg_Flag;          //Gps

uint8_t    GGA_Ini_Flag;
uint8_t    Base_No_Num;
uint16_t   GSA_Length;


//-------------------------------------------
uint8_t   GGA_Rx_Data[20];  //�交�啁�
uint8_t   GGA_Rx_Counter;
uint8_t   GGA_Rx_Max;



uint8_t   m_GSPData_First;
uint16_t  m_GSPData_Num;                    //
uint16_t  m_GSPData_NumB;
uint8_t    Gps_No_Num;
uint16_t   Gps_No_Data_Num;
uint8_t    Gps_No_Flag;

uint16_t   GGARMC_Length;
uint16_t   GGARMCGST_Length;

uint8_t    GGA_Zero_Flag;
uint8_t    RMC_Back_Flag;
//-------------------------------------------
//   SendGps.h
//-------------------------------------------

uint8_t   GGA_Data[20];   
uint8_t   GGB_Data[8];        
uint8_t   GGS_Data[15];  

uint8_t   IMU_Send_OK;   //IMU�唳��瘥?
uint8_t   GPS_Send_OK;   //GPS���唳��瘥?
uint8_t   GNSS_Send_OK;  //蝏�撖潸�唳��瘥?
uint8_t   GSA_Send_OK;   //GSA���唳��瘥?
uint8_t   ATT_Send_OK;   //ATT���唳��瘥?

uint8_t   GNSS_Send_OK_Flag;

uint8_t   PackDebug_Flag;
uint8_t   PackDebug_Time;
uint8_t   DMA_Send_Kind;   //Kind=1,�蛹GPS;Kind=2嚗�銝截MU�唳;Kind=3嚗�銝慘NSS�唳;
uint16_t  DMA_Tx_Num;


//-------------------------------------------
//   GetNass.h
//-------------------------------------------

uint8_t  GpsDriverImuFlag;       //Gps�唳�瑕�
uint8_t  GpsDriverAnalyseFlag;   //Gps閫��摰�...
uint8_t  GpsDriverSendFlag;      //Gps���....


uint8_t  GpsInsGetFlag;        //GpsIns�瑕�
uint8_t  Gnss_Get_Flag;        //�瑕�蝏�撖潸 ......
uint8_t  ZeroSecondFlag;

uint8_t  Gnss_Up_Flag;         //



//-----------------------------------------------------------------
// "commdata.h"
//-----------------------------------------------------------------
uint8_t   TDM_TX_Data[400];	 //�蝏? ��澶GA+RMC
uint8_t   TDM_Rx_Data[60];	 //�交蝏  ��踹��漲霈?

uint8_t   High_DGet_Flag;       //憸�霈曄蔭�
uint8_t   Buat_DGet_Flag;       //��霈曄蔭�
uint8_t   GNGA_DGet_Flag;       //Gps�瑕���
uint8_t   ZDA_DGet_Flag;        //Gps�瑕���
uint8_t   SGPS_DGet_Flag;        //Gps�瑕���

uint8_t   BD_DGet_Flag;         //BD霈曄蔭��
uint8_t   BD_GGet_Flag;         //BD�瑕���

uint8_t   GST_TGet_Flag;        //���喲GST
uint8_t   ATT_DGet_Flag;        //���喲ATT
uint8_t   INS_DGet_Flag;        //���喲INS
uint8_t   GSV_DGet_Flag;        //���喲GSV

//----------------------------------------------------------
uint8_t    ANG_DGet_Flag;        //��摮閫漲
uint8_t    ANG_FGet_Flag;        //��揣閫漲

uint8_t    RSE_DGet_Flag;        //��揣
uint8_t    RSE_FGet_Flag;        //��揣
uint8_t    RSE_SGet_Flag;        //��揣

double     Lat_Back;             //蝏漲
double     Lon_Back;             //蝏游漲

int16_t     Pitch_Back;           //靽臭趕閫?
int16_t     Roll_Back;            //璅芣�閫?
int16_t     Head_Back;            //�孵�閫?

uint8_t     NorthSouthBack;       //
uint8_t     WestEastBack;         //


uint8_t     ANG_Lock_Flag;        //��
uint8_t     ANG_Kind_Flag;        //�嗆�0:�芷�� 1嚗摰��?

uint8_t     RSE_FGet_Flag;        //��揣閫漲
uint8_t     POS_First_Flag;       //

int16_t    MisAngleData;         //摰�閫漲

uint8_t    Ini0_KindBack;        //
int16_t    Ini0_RollBack;        //
int16_t    Ini0_PitchBack;       //

uint8_t    Ini0_KindNew;        //
int16_t    Ini0_RollNew;        //
int16_t    Ini0_PitchNew;       //
//----------------------------------------------------------
uint8_t    Ins_Go_Flag;


uint8_t    Uart_Kind;           //
uint8_t    User_Kind;           //�冽蝐餃�

uint8_t   Debug_Flag;          //靚�....
uint8_t   Up_On_Flag;          //頧臭辣��? 1.01�?           //Store    Command_Sort:0A
uint8_t   Data_Sort;		       //�唳蝐餃�


uint8_t   Pass_Rx_Data[10];  //�交�啁�
uint8_t   Pass_Rx_Counter;
uint8_t   Pass_Rx_Max;

uint8_t   Up_Rx_Data[10];      //
uint8_t   Up_Rx_Counter;      // 
uint8_t   Up_Rx_Max;          // 

uint8_t   Key_Rx_Data[20];  //�交�啁�
uint8_t   Key_Rx_Counter;
uint8_t   Key_Rx_Max;


uint8_t  IMUTGetFlag;


uint8_t   Command_Tx_Max;	   //���憭批��葵�堆�
uint8_t   Command_Rx_Max;	   //�亙���憭批��葵�堆�

uint8_t   Comm_Sort;		       //�賭誘蝐餃�
uint8_t   Data_Com_Flag;      //1--�唳嚗?--�賭誘

uint8_t   AHRSComFlag;		     //
uint8_t   COM_Flag;		       //銝脣��敹�

//-----------------------------------------------------------------
// "EncryptID.h"
//-----------------------------------------------------------------
uint8_t   CPU_ID_Data[25];

//-----------------------------------------------------------------
//"stmflash.h"
//-----------------------------------------------------------------
uint8_t  Flash_Wing_Flag;     //Flash���雿?
uint8_t  Flash_Wirte_Flag; 

uint8_t   Data_Flash_W_Flag;   //��雿?---�臬蝟餌���

uint8_t   Update_One;
uint8_t   Update_Two;

uint8_t   Update_Flash_W_Flag;
uint8_t   MAngle_Flash_W_Flag;

uint16_t  SVersionH;           // 頧臭辣��?撟港遢
uint16_t  SVersionL;           // 頧臭辣��?�遢
uint16_t  HVersionH;           // 蝖砌辣��?撟港遢
uint16_t  HVersionL;           // 蝖砌辣��?�遢
uint16_t  HVersionS;           // 蝖砌辣��?摨

uint8_t  Flash_Rdata[150];	   //�扇嚗摮�唳銝芣�����蛹�閬��Ｖ蛹uint32
uint8_t  Flash_Wdata[150];
//----------------------------------------------------------------

FLOAT32  YZ_Acce_Deta;
FLOAT32  YZ_Dece_Deta;

FLOAT32  YZ_Turn_Deta;
FLOAT32  YZ_Turn_Anlge;

FLOAT32  YZ_Lane_Deta;
FLOAT32  YZ_Lane_Anlge;

FLOAT32  YZ_Coll_Deta;

FLOAT32  YZ_Stab_Deta;
FLOAT32  YZ_Stab_Time;

FLOAT32  YZ_Att_Deta;

FLOAT32  YZ_Att_Min_Deta;
FLOAT32  YZ_Att_Max_Deta;

//-----kalman-----------------------------
float  Scale;
float  pi;

//***********************************************************************************
//
//***********************************************************************************

IMU_DATA_T    IMUDataBuffer;	 //gty IMU�������霂亙�....
GNSS_DATA_T   GNSSDataBuffer;  //gty GPS�������霂亙�....

PIMU_DATA_T   pImuData;        //gty ��IMU....
PGNSS_DATA_T  pGnssData;       //gty ��GPS....

OUTPUT_INFO_T GINavResult;     //gty 蝏�撖潸颲���縑�?

GSAV_DATA_T   g_GsavInfo;       //Gty  �瑕�GSA,GSV靽⊥....

BOOL IMUDataReady;             //gty IMU�唳�臬��憟踝�憒���憟踝�瘥活�典�嚗��?
BOOL GNSSDataReady;            //gty GNSS�唳�臬��憟踝�憒���憟踝�瘥活�典�嚗��?

BOOL GNSSDataGetFlag;          //gty �其��唳�郊
BOOL IMUDataGetFlag;           //gty �其��唳�郊  

//***********************************************************************************
//
//***********************************************************************************

uint8_t error_code;
uint8_t Ddx_data[8];		 
uint8_t Ttx_data[8];

void Data_Ini(void); 
void Data_Synch(void);
void ApluCode(void);

extern unsigned char alpuc_process(unsigned char *, unsigned char *);
//瘚�PPS�?----------------------------------------
int time1;
int time0;

//瘚�雿��-----------------------------------
uint8_t Lowpow_Flag;
uint8_t Lowpow_Sleep;
int Choose_Lowpow_Flag=2;//1:Sleep mode  3:Standby mode
uint8_t Lowpow_Check;
char lowpower_timer_start;
char lowpower_start;
int Lowpow_time;
uint8_t lowpow;
 
//EEPROM瘚�雿輻----------------------------------
const u8 TEXT_Buffer[]={0x12,0x13,0x14,0x15,0x16,0x22,0x17,0x18,0x19,0x1A};
#define SIZE sizeof(TEXT_Buffer)
u8 datatemp[20];
u8 testcheck=0;
//tes source insight-----------pull test---------
int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);		   //20140524

	//---------------------------
	Update_One=0;   
  Update_Two=0; 	
	Update_Flash_INI(); 
	STMFLASH_Write(ADDR_FLASH_SECTOR_2 ,(u32*)Flash_Wdata,UpData_Buff_Size);
	//---------------------------

	Data_Ini();	             //敹◆�求ata_Flash_Decode銋�嚗�霂MU_Kind�舀�圄lash餈�霂...
	Check_Lock_Code();
	
	delay_init(84); 
	Set_System();
	
	//---------------------------
	delay_ms(500);	
	ApluCode();              //��....餈葵敹◆���袁ey_Flash_Decode銋�...憒�銝�撖���餈�蝏�撖潸..
	
	MisAngle_Flash_Decode(); //
	Data_Flash_Decode();     //蝟餌��閫��+�唳靽

	//---------------------------
	USART2_BodRate=115200;
  USART_Configuration();

	
	if(User_Kind!=5)
	SendVersion();          // ���?.	
		
	Acc_Ini();              // �漲霈?
	Gyr_Ini();              // ��箔貌
	
	//BMI055A_Ini();		      //�漲霈∪�憪�
	//BMI055G_Ini();		      //��箔貌���?
	
	//BMI160瘚����?
	BMI160_Ini();	
	delay_ms(10);
		
	GINavInit();            //蝏�撖潸���?..
		
	NVIC_Configuration();		// NVIC�蔭
	
 //------------------------------------------------
	GPS_Rate_Config();      // GPS  �蔭
	GpsConfig2();	
	
 //------------------------------------------------	
 //	IWDG_Init(4,2000);	    // 霈曄蔭1S��?
	
	DIni_Flag=1;
	
	//雿��?----------------------
	Interrupt_Any_motion();	//璉瘚��?
	
	//EEPROM瘚��?-------------------------------
	 while( EEPROM_Check()!=0){};//璉瘚EROM餈��
   EEPROM_Write(0,(u8*)TEXT_Buffer,SIZE);
		 
  while (1)
  {
		//�∠�璅∪�瘚�------------------------------OK
		if(Lowpow_Sleep==1)//�og sleep憭�
		{
			  Lowpow_Sleep=0;
			  __set_FAULTMASK(1);      // �喲��葉�?
				NVIC_SystemReset();      // 憭�	
		}
		if(Lowpow_Flag)//銝脣�賭誘log lowpo餈雿��?
		{
			 Lowpow_Flag = 0;
			 //stop mode-----------------------璅∪�鈭?PA0 160�NT1�日�
			 if(Choose_Lowpow_Flag==2)
       {
				 
				 
				 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
				 PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI); 
				 __nop();
				 __nop();
		   } 
			 //Standby mode--------------------璅∪�銝?
			 else if(Choose_Lowpow_Flag==3)
			 {   
					// GPIO_SetBits(GPIOA,GPIO_Pin_15);//�袋BLOX
					 
					 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA  |RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOC,DISABLE); 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |RCC_APB1Periph_TIM3  |RCC_APB1Periph_TIM2 |RCC_APB1Periph_SPI2,DISABLE);
					 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG |RCC_APB2Periph_USART1 |RCC_APB2Periph_USART6|RCC_APB2Periph_SPI1,DISABLE);
					 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
					 PWR_ClearFlag(PWR_FLAG_WU);//瘜剁�敹◆皜
					 PWR_WakeUpPinCmd(ENABLE);
					 PWR_EnterSTANDBYMode();
			 }
		 }	 
		 
		//160瘚�-------------------------------------OK
//		Get_BMI160G_Data();	//1-�瑕���箔貌�唳
//		Get_Rate_Data();    //1-�瑕��箔���箔貌��摨虫縑�胯?
//		Get_Yaw_Data();     //1-�瑕��箔���箔貌�尿��潘�	

//		Get_BMI160A_Data();//2-�瑕��漲�唳
//		Get_Acc_XYZ();	   //2-�瑕��漲�啣潘�
//		Get_Angle_XYZ();   //2-�瑕��箔��漲�尿��潘�	
		 
     //EEPROM-FM24C512瘚�-----------------------OK
//			EEPROM_Read(0,datatemp,SIZE);//eeprom瘚�
//			testcheck=EEPROM_ReadOneByte(65535);
//		  delay_ms(500);
		 
		 if(Up_On_Flag==0)
		 {	
			GINavProc(&GINavResult);     //蝏�撖潸憭�....
			Pack_GnssData();             //�渡�GNSS�唳....
			Send_GnssData();		         //�NSS�唳....	
			Send_GSAData();              //�瑕��唳...			 
		 }	 
		 else if(Up_On_Flag==1)
		 {
			SendData(); 
			Com2SendData();			
		 }
     else
		 {
	    Data_Flash_Save();
		  Update_Flash_Save();		 
	    MisAngle_Flash_Save();			 
     }		 

  }	
}


void Data_Ini(void)
{
  //****************************************************************************************
	//  �虜嚗�撣賊�閬�嚗�嚗�
	//****************************************************************************************
	DIni_Flag=0;
	//------------------------------------------------------
	//   銝�祉�? 9600   1HZ   GGA,RMC GSA GSV
	//------------------------------------------------------
	//瘜冽�flash暺恕霈曄蔭,餈��挽蝵格瓷�嚗?
	IMU_Kind     = 2;             //1: 憭拇���          2: 1216璅∪�  X頧湔��?..(憭拙極MTK�)  3:XXX            4嚗?          5嚗?216璅∪� X頧湔��?..  6:1�對�頞�+�喃�嚗噫�? 7嚗mouse(��? 8嚗?��嚗之��
	User_Kind    = 7;             //1: �桅恥�?         2嚗�敺?                             3:�?          4:暽啁        5: �答                 6: �漪摰Ｘ             7: 憭拙極瘚�       8嚗�鈭��?     9:�菜?

	Debug_Flag   = 0;	            //0嚗極雿�?         1嚗�霂�? 115200			

	GpsKind      = 1;             //1: Ulbox             2: Mtk
	ProductKind  = 1;             //1: 璅∪�              2嚗mosue
	
	//****************************************************************************************
		
	Uart_Kind   = 2;              //1:Uart1->�交GPS Uart2->�冽  2嚗�銋?
	
	Up_On_Flag  = 0;              //0: 撖潸�悅                   1嚗�霂�霈?
	Gps_Fren_Kind  = 1;           //5: 暺恕5hz...銝�血��?HZ颲嚗�Sim_Mode=0....頧祆銝箸�埂PS撽勗.....
	//--------------------------------------
	//---------------------------------------
  
	Diff_First_Flag=0;
	
	BaseTime=10;                   //INS=10hz,BaseTime=20;  INS=20hz,BaseTime=10ms
	
	DetaTime=1000/5/BaseTime;     //�?暺恕55HZ颲嚗�5s->20......
	
//----------------------
  USART2_BodRate=115200;
//----------------------
  Pass_Rx_Max =0;	
  GGA_Rx_Max  =0;
//----------------------	
  IMU_Flag   = 0; 
	T1times    = 0;
	//----------------------
	
	pi             =  3.1415926;
  Scale          =  180/pi;


	Flash_Wing_Flag   = 0;	

  T1times  = 0;
  SaveTime = 0;
	PackTime = 0;
  SendTime = 0;
 
  T2times=0;
  T3times=0;

 //---------------------------------
 //
 //---------------------------------
  GNSSDataReady   = FALSE;
  IMUDataReady    = FALSE;
	GNSSDataGetFlag = FALSE;
	IMUDataGetFlag  = FALSE; 


 //---------------------------------
 //
 //---------------------------------
	
	GPS_Send_OK     = 0;    //2--�PS�唳摰�
	GNSS_Send_OK    = 0;    //3--�NSS�唳摰�
	IMU_Send_OK     = 0;    //4--�MU�唳摰�
	DMA_Send_Kind   = 0;    //Kind=1,�蛹GPS嚗ind=2嚗�銝截MU�唳....
	
	
 //---------------------------------
 //
 //---------------------------------
 Gps_No_Num=0;
 Gps_No_Data_Num=0;
 Gps_No_Flag=1;

}


///-----------------------------
//void Aplu-Code(void)
///-----------------------------
void ApluCode(void)
{
	uint8_t i;


	
	Ttx_data[0]=_alpu_rand();
	Ttx_data[1]=_alpu_rand();
	Ttx_data[2]=_alpu_rand();
	Ttx_data[3]=_alpu_rand();
	Ttx_data[4]=_alpu_rand();
	Ttx_data[5]=_alpu_rand();
	Ttx_data[6]=_alpu_rand();
	Ttx_data[7]=_alpu_rand();

	for(i=0; i<8; i++)
	{	
		Ddx_data[i] = 0;
	}
	
	error_code = alpuc_process(Ttx_data,Ddx_data);  
  

	
}



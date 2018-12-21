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

//uint8_t  BMI055A_Rx[6];	   //���ռ��ٶȼ�ԭʼ����
uint8_t  BMI160A_Rx[6];
int16_t	 ACC_X;			       //���ٶȼ�X������
int16_t  ACC_Y;			       //���ٶȼ�Y������
int16_t  ACC_Z;			       //���ٶȼ�Z������

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

int16_t  intPitchAngle;	   //���ڼ��ٶȼƵĸ�����
int16_t  intRollAngle;	   //���ڼ��ٶȼƵĺ����
int16_t  intYawAngle;      //���ڽ��ٶȵĺ����

//-----------------------------------------
//    GetGyr.h
//-----------------------------------------
//uint8_t BMI055G_Rx[6];	  //
uint8_t BMI160G_Rx[6];


int16_t  Gyr_X;			      //������X������
int16_t  Gyr_Y;			      //������Y������
int16_t  Gyr_Z;			      //������Z������

int16_t  GyrBias_X;       // �����ǵĸ�����ƫ��ʵʱ����   
int16_t  GyrBias_Y;       // �����ǵĸ�����ƫ��ʵʱ����   
int16_t  GyrBias_Z;       // �����ǵĸ�����ƫ��ʵʱ����

int16_t  GyrScale_X;      // �����ǵ�X��������  store
int16_t  GyrScale_Y;      // �����ǵ�Y��������  store
int16_t  GyrScale_Z;      // �����ǵ�Z��������  store 

int16_t  XGyrBias;        // �����ǵĸ�����ƫ��   store
int16_t  YGyrBias;        // �����ǵĸ�����ƫ��   store
int16_t  ZGyrBias;        // �����ǵĸ�����ƫ��   store

int16_t  XGyrScale;       // �����ǵ�X��������  store
int16_t  YGyrScale;       // �����ǵ�Y��������  store
int16_t  ZGyrScale;       // �����ǵ�Z��������  store 

float  floPitchGyr;		  //???��X
float  floRollGyr;		  //???��Y
float  floYawGyr;		    //???��Z

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
int16_t   IMU_SData[51][8];     //���ڴ洢����.....
uint16_t  DAT_SData[21][4];     //

uint8_t   m_IMUData[300];       //
uint8_t   m_IMUDataB[300];      //

uint8_t   IMUTC_Data[20];       //UTCʱ��....
uint8_t   IMU_DATAT[60];        //���ڴ��IMU����
//--------------------------------------------------

uint8_t   FrameFirst;

uint8_t   BaseTime;        //
uint8_t   DetaTime;        //

uint8_t   SaveTime;        //������������....
uint8_t   SendTime;        //SendTime��SaveTime֮����Ҫ�Ƚϣ�����һ��ע�ⲻҪ����޸�....
uint8_t   PackTime;        //PackTime��SaveTime֮����Ҫ�Ƚϣ�����һ��ע�ⲻҪ����޸�....

uint8_t   IMU_Send_Flag;

uint16_t  IMU_DMA_Num;

uint16_t  IMDMStime;

uint8_t   IMDHour;						//	 UTCʱ�䣺ʱ
uint8_t   IMDMinute;					//	 ��
uint8_t   IMDSecond;					//	 ��
uint8_t   IMDMSecond;        //   ����

uint8_t   IMU_Flag;          //
uint8_t   Ins_Go_Flag;
//--------------------------------------
uint8_t    T1times;
uint16_t   T2times;
uint16_t   T3times;

uint8_t    Change_Data[50];
uint8_t    IMU_Num;           //��ȡIMU���ݵĴ���
//-------------------------------------------
//   GetGps.h
//-------------------------------------------
uint8_t   m_GSPData[NP_ALL_DATA_LEN];       // NMEA data     900  GGA+RMC+GSA+GSV+GST
uint8_t   m_GSPDataB[NP_ALL_DATA_LEN];      // NMEA data     900  GSA+GSV+GST

uint8_t   m_GPSpData[NP_MAX_DATA_LEN];       // NMEA data     NMEA����
uint8_t   m_RMCpData[NP_MAX_DATA_LEN];      // NMEA data     NMEA����

uint8_t     GGA_Get_Flag;        //Gps��ñ�־
uint8_t     RMC_Get_Flag;        //Gps��ñ�־
uint8_t     VTG_Get_Flag;        //Gps��ñ�־
uint8_t     GST_Get_Flag;        //Gps��ñ�־
uint8_t   	GSA_Get_Flag;	       //Gps��ñ�־
uint8_t  	  GSV_Get_Flag;        //Gps��ñ�־


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

GPGGA_DATA  GPGGAData;				    //�ṹ�壬�洢����Э���е�����
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
uint8_t   GGA_Rx_Data[20];  //��������
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

uint8_t   IMU_Send_OK;   //IMU���ݷ������
uint8_t   GPS_Send_OK;   //GPSԭʼ���ݷ������
uint8_t   GNSS_Send_OK;  //��ϵ������ݷ������
uint8_t   GSA_Send_OK;   //GSAԭʼ���ݷ������
uint8_t   ATT_Send_OK;   //ATTԭʼ���ݷ������

uint8_t   GNSS_Send_OK_Flag;

uint8_t   PackDebug_Flag;
uint8_t   PackDebug_Time;
uint8_t   DMA_Send_Kind;   //Kind=1,��ΪGPS;Kind=2����ΪIMU����;Kind=3����ΪGNSS����;
uint16_t  DMA_Tx_Num;


//-------------------------------------------
//   GetNass.h
//-------------------------------------------

uint8_t  GpsDriverImuFlag;       //Gps���ݻ��
uint8_t  GpsDriverAnalyseFlag;   //Gps�������...
uint8_t  GpsDriverSendFlag;      //Gps���´��....


uint8_t  GpsInsGetFlag;        //GpsIns���
uint8_t  Gnss_Get_Flag;        //�����ϵ��� ......
uint8_t  ZeroSecondFlag;

uint8_t  Gnss_Up_Flag;         //



//-----------------------------------------------------------------
// "commdata.h"
//-----------------------------------------------------------------
uint8_t   TDM_TX_Data[400];	 //��������  �GGA+RMC
uint8_t   TDM_Rx_Data[60];	 //��������  ����ٶȼ�

uint8_t   High_DGet_Flag;       //Ƶ�����ò���
uint8_t   Buat_DGet_Flag;       //�������ò���
uint8_t   GNGA_DGet_Flag;       //Gps��ñ�־
uint8_t   ZDA_DGet_Flag;        //Gps��ñ�־
uint8_t   SGPS_DGet_Flag;        //Gps��ñ�־

uint8_t   BD_DGet_Flag;         //BD���ñ�־
uint8_t   BD_GGet_Flag;         //BD��ñ�־

uint8_t   GST_TGet_Flag;        //�򿪹ر�GST
uint8_t   ATT_DGet_Flag;        //�򿪹ر�ATT
uint8_t   INS_DGet_Flag;        //�򿪹ر�INS
uint8_t   GSV_DGet_Flag;        //�򿪹ر�GSV

//----------------------------------------------------------
uint8_t    ANG_DGet_Flag;        //��Ч�洢�Ƕ�
uint8_t    ANG_FGet_Flag;        //���������Ƕ�

uint8_t    RSE_DGet_Flag;        //��������
uint8_t    RSE_FGet_Flag;        //��������
uint8_t    RSE_SGet_Flag;        //��������

double     Lat_Back;             //����
double     Lon_Back;             //ά��

int16_t     Pitch_Back;           //������
int16_t     Roll_Back;            //�����
int16_t     Head_Back;            //�����

uint8_t     NorthSouthBack;       //
uint8_t     WestEastBack;         //


uint8_t     ANG_Lock_Flag;        //����
uint8_t     ANG_Kind_Flag;        //״̬��0:����Ӧ�汾 1���̶��汾

uint8_t     RSE_FGet_Flag;        //���������Ƕ�
uint8_t     POS_First_Flag;       //

int16_t    MisAngleData;         //��װ�Ƕ�

uint8_t    Ini0_KindBack;        //
int16_t    Ini0_RollBack;        //
int16_t    Ini0_PitchBack;       //

uint8_t    Ini0_KindNew;        //
int16_t    Ini0_RollNew;        //
int16_t    Ini0_PitchNew;       //
//----------------------------------------------------------
uint8_t    Ins_Go_Flag;


uint8_t    Uart_Kind;           //
uint8_t    User_Kind;           //�û�����

uint8_t   Debug_Flag;          //����....
uint8_t   Up_On_Flag;          //����汾��  1.01��            //Store    Command_Sort:0A
uint8_t   Data_Sort;		       //��������


uint8_t   Pass_Rx_Data[10];  //��������
uint8_t   Pass_Rx_Counter;
uint8_t   Pass_Rx_Max;

uint8_t   Up_Rx_Data[10];      //
uint8_t   Up_Rx_Counter;      // 
uint8_t   Up_Rx_Max;          // 

uint8_t   Key_Rx_Data[20];  //��������
uint8_t   Key_Rx_Counter;
uint8_t   Key_Rx_Max;


uint8_t  IMUTGetFlag;


uint8_t   Command_Tx_Max;	   //���͵�����ֽڸ�����
uint8_t   Command_Rx_Max;	   //���ܵ�����ֽڸ�����

uint8_t   Comm_Sort;		       //��������
uint8_t   Data_Com_Flag;      //1--���ݣ�2--����

uint8_t   AHRSComFlag;		     //
uint8_t   COM_Flag;		       //���ڷ��ͱ�־λ

//-----------------------------------------------------------------
// "EncryptID.h"
//-----------------------------------------------------------------
uint8_t   CPU_ID_Data[25];

//-----------------------------------------------------------------
//"stmflash.h"
//-----------------------------------------------------------------
uint8_t  Flash_Wing_Flag;     //Flashд���־λ
uint8_t  Flash_Wirte_Flag; 

uint8_t   Data_Flash_W_Flag;   //��־λ----����ϵͳ����д��

uint8_t   Update_One;
uint8_t   Update_Two;

uint8_t   Update_Flash_W_Flag;
uint8_t   MAngle_Flash_W_Flag;

uint16_t  SVersionH;           // ����汾�� ���
uint16_t  SVersionL;           // ����汾�� �·�
uint16_t  HVersionH;           // Ӳ���汾�� ���
uint16_t  HVersionL;           // Ӳ���汾�� �·�
uint16_t  HVersionS;           // Ӳ���汾�� ���

uint8_t  Flash_Rdata[150];	   //�мǣ��Ǵ洢���ݸ������ı�����Ϊ��Ҫ�任Ϊuint32
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

IMU_DATA_T    IMUDataBuffer;	 //gty IMU����ָ���ָ��õ�ַ....
GNSS_DATA_T   GNSSDataBuffer;  //gty GPS����ָ���ָ��õ�ַ....

PIMU_DATA_T   pImuData;        //gty ָ��IMU....
PGNSS_DATA_T  pGnssData;       //gty ָ��GPS....

OUTPUT_INFO_T GINavResult;     //gty ��ϵ��������������Ϣ

GSAV_DATA_T   g_GsavInfo;       //Gty  ���GSA,GSV��Ϣ....

BOOL IMUDataReady;             //gty IMU�����Ƿ�׼���ã����׼���ã�ÿ�����꣬����
BOOL GNSSDataReady;            //gty GNSS�����Ƿ�׼���ã����׼���ã�ÿ�����꣬����

BOOL GNSSDataGetFlag;          //gty ��������ͬ��
BOOL IMUDataGetFlag;           //gty ��������ͬ��  

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
//����PPS��-----------------------------------------
int time1;
int time0;

//���Ե͹�����-----------------------------------
uint8_t Lowpow_Flag;
uint8_t Lowpow_Sleep;
int Choose_Lowpow_Flag=2;//1:Sleep mode  3:Standby mode
uint8_t Lowpow_Check;
char lowpower_timer_start;
char lowpower_start;
int Lowpow_time;
uint8_t lowpow;
 
//EEPROM����ʹ��----------------------------------
const u8 TEXT_Buffer[]={0x12,0x13,0x14,0x15,0x16,0x22,0x17,0x18,0x19,0x1A};
#define SIZE sizeof(TEXT_Buffer)
u8 datatemp[20];
u8 testcheck=0;

int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);		   //20140524

	//---------------------------
	Update_One=0;   
  Update_Two=0; 	
	Update_Flash_INI(); 
	STMFLASH_Write(ADDR_FLASH_SECTOR_2 ,(u32*)Flash_Wdata,UpData_Buff_Size);
	//---------------------------

	Data_Ini();	             //������Data_Flash_Decode֮ǰ����֤IMU_Kind�Ǹ���Flash��������...
	Check_Lock_Code();
	
	delay_init(84); 
	Set_System();
	
	//---------------------------
	delay_ms(500);	
	ApluCode();              //����....���������ǰ��Key_Flash_Decode֮ǰ...��������ܣ��򲻽�����ϵ���..
	
	MisAngle_Flash_Decode(); //
	Data_Flash_Decode();     //ϵͳ��������+���ݱ���

	//---------------------------
	USART2_BodRate=115200;
  USART_Configuration();

	
	if(User_Kind!=5)
	SendVersion();          // ���Ͱ汾..	
		
	Acc_Ini();              // ���ٶȼ�
	Gyr_Ini();              // ������
	
	//BMI055A_Ini();		      //���ٶȼƳ�ʼ��
	//BMI055G_Ini();		      //�����ǳ�ʼ��
	
	//BMI160���Գ�ʼ��
	BMI160_Ini();	
	delay_ms(10);
		
	GINavInit();            //��ϵ�����ʼ��...
		
	NVIC_Configuration();		// NVIC����
	
 //------------------------------------------------
	GPS_Rate_Config();      // GPS  ����
	GpsConfig2();	
	
 //------------------------------------------------	
 //	IWDG_Init(4,2000);	    // ����1S���Ź�
	
	DIni_Flag=1;
	
	//�͹���-----------------------
	Interrupt_Any_motion();	//����˶�
	
	//EEPROM����д--------------------------------
	 while( EEPROM_Check()!=0){};//���EEROM���ӳɹ�
   EEPROM_Write(0,(u8*)TEXT_Buffer,SIZE);
		 
  while (1)
  {
		//˯��ģʽ����------------------------------OK
		if(Lowpow_Sleep==1)//����log sleep��λ
		{
			  Lowpow_Sleep=0;
			  __set_FAULTMASK(1);      // �ر������ж�
				NVIC_SystemReset();      // ��λ	
		}
		if(Lowpow_Flag)//��������log lowpo����͹���
		{
			 Lowpow_Flag = 0;
			 //stop mode-----------------------ģʽ�� PA0 160��INT1����
			 if(Choose_Lowpow_Flag==2)
       {
				 
				 
				 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
				 PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI); 
				 __nop();
				 __nop();
		   } 
			 //Standby mode--------------------ģʽ��
			 else if(Choose_Lowpow_Flag==3)
			 {   
					// GPIO_SetBits(GPIOA,GPIO_Pin_15);//��UBLOX
					 
					 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA  |RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOC,DISABLE); 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |RCC_APB1Periph_TIM3  |RCC_APB1Periph_TIM2 |RCC_APB1Periph_SPI2,DISABLE);
					 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG |RCC_APB2Periph_USART1 |RCC_APB2Periph_USART6|RCC_APB2Periph_SPI1,DISABLE);
					 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
					 PWR_ClearFlag(PWR_FLAG_WU);//ע���������
					 PWR_WakeUpPinCmd(ENABLE);
					 PWR_EnterSTANDBYMode();
			 }
		 }	 
		 
		//160����-------------------------------------OK
//		Get_BMI160G_Data();	//1-�������������
//		Get_Rate_Data();    //1-��û��������ǵĽǶ���Ϣ��
//		Get_Yaw_Data();     //1-��û��������ǵ���̬��ֵ��	

//		Get_BMI160A_Data();//2-��ü��ٶ�����
//		Get_Acc_XYZ();	   //2-��ü��ٶ���ֵ��
//		Get_Angle_XYZ();   //2-��û��ڼ��ٶȵ���̬��ֵ��	
		 
     //EEPROM-FM24C512����-----------------------OK
//			EEPROM_Read(0,datatemp,SIZE);//eeprom����
//			testcheck=EEPROM_ReadOneByte(65535);
//		  delay_ms(500);
		 
		 if(Up_On_Flag==0)
		 {	
			GINavProc(&GINavResult);     //��ϵ�������....
			Pack_GnssData();             //����GNSS����....
			Send_GnssData();		         //����GNSS����....	
			Send_GSAData();              //�������...			 
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
	//  �ǳ����ǳ���Ҫ����������
	//****************************************************************************************
	DIni_Flag=0;
	//------------------------------------------------------
	//   һ���û�  9600   1HZ   GGA,RMC GSA GSV
	//------------------------------------------------------
	//ע��flashĬ������,���������û���ã�
	IMU_Kind     = 2;             //1: ���Ǳ���          2: 1216ģ��  X�ᳯ��...(�칤MTK�汾)  3:XXX            4��           5��1216ģ�� X�ᳯǰ...  6:1�㣨��ǰ+���ϣ�����  7��Gmouse(���ε�) 8��3��װ�����ƣ�
	User_Kind    = 7;             //1: ��ͨ�ͻ�          2��Ԫ��	                             3:��;           4:ӥ�        5: ����                 6: �����ͻ�             7: �칤����       8�����ƻ���      9:����

	Debug_Flag   = 0;	            //0������״̬          1������״̬  115200			

	GpsKind      = 1;             //1: Ulbox             2: Mtk
	ProductKind  = 1;             //1: ģ��              2��Gmosue
	
	//****************************************************************************************
		
	Uart_Kind   = 2;              //1:Uart1->����GPS Uart2->�û�  2����֮
	
	Up_On_Flag  = 0;              //0: ����Э��                   1������Э��	
	Gps_Fren_Kind  = 1;           //5: Ĭ��5hz...һ������5HZ���룬��Sim_Mode=0....ת��Ϊ����GPS����.....
	//--------------------------------------
	//---------------------------------------
  
	Diff_First_Flag=0;
	
	BaseTime=10;                   //INS=10hz,BaseTime=20;  INS=20hz,BaseTime=10ms
	
	DetaTime=1000/5/BaseTime;     //��ֵ Ĭ��55HZ���룬��5s->20......
	
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
	
	GPS_Send_OK     = 0;    //2--����GPS�������
	GNSS_Send_OK    = 0;    //3--����GNSS�������
	IMU_Send_OK     = 0;    //4--����IMU�������
	DMA_Send_Kind   = 0;    //Kind=1,��ΪGPS��Kind=2����ΪIMU����....
	
	
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



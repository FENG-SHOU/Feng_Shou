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

//uint8_t  BMI055A_Rx[6];	   //接收加速度计原始数据
uint8_t  BMI160A_Rx[6];
int16_t	 ACC_X;			       //加速度计X轴数据
int16_t  ACC_Y;			       //加速度计Y轴数据
int16_t  ACC_Z;			       //加速度计Z轴数据

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

int16_t  intPitchAngle;	   //基于加速度计的俯仰角
int16_t  intRollAngle;	   //基于加速度计的横滚角
int16_t  intYawAngle;      //基于角速度的航向角

//-----------------------------------------
//    GetGyr.h
//-----------------------------------------
//uint8_t BMI055G_Rx[6];	  //
uint8_t BMI160G_Rx[6];


int16_t  Gyr_X;			      //陀螺仪X轴数据
int16_t  Gyr_Y;			      //陀螺仪Y轴数据
int16_t  Gyr_Z;			      //陀螺仪Z轴数据

int16_t  GyrBias_X;       // 陀螺仪的俯仰轴偏置实时变量   
int16_t  GyrBias_Y;       // 陀螺仪的俯仰轴偏置实时变量   
int16_t  GyrBias_Z;       // 陀螺仪的俯仰轴偏置实时变量

int16_t  GyrScale_X;      // 陀螺仪的X轴标称因子  store
int16_t  GyrScale_Y;      // 陀螺仪的Y轴标称因子  store
int16_t  GyrScale_Z;      // 陀螺仪的Z轴标称因子  store 

int16_t  XGyrBias;        // 陀螺仪的俯仰轴偏置   store
int16_t  YGyrBias;        // 陀螺仪的俯仰轴偏置   store
int16_t  ZGyrBias;        // 陀螺仪的俯仰轴偏置   store

int16_t  XGyrScale;       // 陀螺仪的X轴标称因子  store
int16_t  YGyrScale;       // 陀螺仪的Y轴标称因子  store
int16_t  ZGyrScale;       // 陀螺仪的Z轴标称因子  store 

float  floPitchGyr;		  //???棗X
float  floRollGyr;		  //???棗Y
float  floYawGyr;		    //???棗Z

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
int16_t   IMU_SData[51][8];     //后期存储数组.....
uint16_t  DAT_SData[21][4];     //

uint8_t   m_IMUData[300];       //
uint8_t   m_IMUDataB[300];      //

uint8_t   IMUTC_Data[20];       //UTC时间....
uint8_t   IMU_DATAT[60];        //用于存放IMU的数
//--------------------------------------------------

uint8_t   FrameFirst;

uint8_t   BaseTime;        //
uint8_t   DetaTime;        //

uint8_t   SaveTime;        //后期数组索引....
uint8_t   SendTime;        //SendTime和SaveTime之间需要比较，所以一定注意不要随便修改....
uint8_t   PackTime;        //PackTime和SaveTime之间需要比较，所以一定注意不要随便修改....

uint8_t   IMU_Send_Flag;

uint16_t  IMU_DMA_Num;

uint16_t  IMDMStime;

uint8_t   IMDHour;						//	 UTC时间：时
uint8_t   IMDMinute;					//	 分
uint8_t   IMDSecond;					//	 秒
uint8_t   IMDMSecond;        //   毫秒

uint8_t   IMU_Flag;          //
uint8_t   Ins_Go_Flag;
//--------------------------------------
uint8_t    T1times;
uint16_t   T2times;
uint16_t   T3times;

uint8_t    Change_Data[50];
uint8_t    IMU_Num;           //读取IMU数据的次数
//-------------------------------------------
//   GetGps.h
//-------------------------------------------
uint8_t   m_GSPData[NP_ALL_DATA_LEN];       // NMEA data     900  GGA+RMC+GSA+GSV+GST
uint8_t   m_GSPDataB[NP_ALL_DATA_LEN];      // NMEA data     900  GSA+GSV+GST

uint8_t   m_GPSpData[NP_MAX_DATA_LEN];       // NMEA data     NMEA数据
uint8_t   m_RMCpData[NP_MAX_DATA_LEN];      // NMEA data     NMEA数据

uint8_t     GGA_Get_Flag;        //Gps获得标志
uint8_t     RMC_Get_Flag;        //Gps获得标志
uint8_t     VTG_Get_Flag;        //Gps获得标志
uint8_t     GST_Get_Flag;        //Gps获得标志
uint8_t   	GSA_Get_Flag;	       //Gps获得标志
uint8_t  	  GSV_Get_Flag;        //Gps获得标志


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

GPGGA_DATA  GPGGAData;				    //结构体，存储各个协议中的内容
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
uint8_t   GGA_Rx_Data[20];  //接收数组
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

uint8_t   IMU_Send_OK;   //IMU数据发送完毕
uint8_t   GPS_Send_OK;   //GPS原始数据发送完毕
uint8_t   GNSS_Send_OK;  //组合导航数据发送完毕
uint8_t   GSA_Send_OK;   //GSA原始数据发送完毕
uint8_t   ATT_Send_OK;   //ATT原始数据发送完毕

uint8_t   GNSS_Send_OK_Flag;

uint8_t   PackDebug_Flag;
uint8_t   PackDebug_Time;
uint8_t   DMA_Send_Kind;   //Kind=1,则为GPS;Kind=2，则为IMU数据;Kind=3，则为GNSS数据;
uint16_t  DMA_Tx_Num;


//-------------------------------------------
//   GetNass.h
//-------------------------------------------

uint8_t  GpsDriverImuFlag;       //Gps数据获得
uint8_t  GpsDriverAnalyseFlag;   //Gps解析完成...
uint8_t  GpsDriverSendFlag;      //Gps重新打包....


uint8_t  GpsInsGetFlag;        //GpsIns获得
uint8_t  Gnss_Get_Flag;        //获得组合导航 ......
uint8_t  ZeroSecondFlag;

uint8_t  Gnss_Up_Flag;         //



//-----------------------------------------------------------------
// "commdata.h"
//-----------------------------------------------------------------
uint8_t   TDM_TX_Data[400];	 //发送数组  最长GGA+RMC
uint8_t   TDM_Rx_Data[60];	 //接收组数  最长加速度计

uint8_t   High_DGet_Flag;       //频率设置参数
uint8_t   Buat_DGet_Flag;       //速率设置参数
uint8_t   GNGA_DGet_Flag;       //Gps获得标志
uint8_t   ZDA_DGet_Flag;        //Gps获得标志
uint8_t   SGPS_DGet_Flag;        //Gps获得标志

uint8_t   BD_DGet_Flag;         //BD设置标志
uint8_t   BD_GGet_Flag;         //BD获得标志

uint8_t   GST_TGet_Flag;        //打开关闭GST
uint8_t   ATT_DGet_Flag;        //打开关闭ATT
uint8_t   INS_DGet_Flag;        //打开关闭INS
uint8_t   GSV_DGet_Flag;        //打开关闭GSV

//----------------------------------------------------------
uint8_t    ANG_DGet_Flag;        //有效存储角度
uint8_t    ANG_FGet_Flag;        //重新搜索角度

uint8_t    RSE_DGet_Flag;        //重新搜索
uint8_t    RSE_FGet_Flag;        //重新搜索
uint8_t    RSE_SGet_Flag;        //重新搜索

double     Lat_Back;             //经度
double     Lon_Back;             //维度

int16_t     Pitch_Back;           //俯仰角
int16_t     Roll_Back;            //横滚角
int16_t     Head_Back;            //方向角

uint8_t     NorthSouthBack;       //
uint8_t     WestEastBack;         //


uint8_t     ANG_Lock_Flag;        //锁定
uint8_t     ANG_Kind_Flag;        //状态：0:自适应版本 1：固定版本

uint8_t     RSE_FGet_Flag;        //重新搜索角度
uint8_t     POS_First_Flag;       //

int16_t    MisAngleData;         //安装角度

uint8_t    Ini0_KindBack;        //
int16_t    Ini0_RollBack;        //
int16_t    Ini0_PitchBack;       //

uint8_t    Ini0_KindNew;        //
int16_t    Ini0_RollNew;        //
int16_t    Ini0_PitchNew;       //
//----------------------------------------------------------
uint8_t    Ins_Go_Flag;


uint8_t    Uart_Kind;           //
uint8_t    User_Kind;           //用户类型

uint8_t   Debug_Flag;          //调试....
uint8_t   Up_On_Flag;          //软件版本号  1.01版            //Store    Command_Sort:0A
uint8_t   Data_Sort;		       //数据类型


uint8_t   Pass_Rx_Data[10];  //接收数组
uint8_t   Pass_Rx_Counter;
uint8_t   Pass_Rx_Max;

uint8_t   Up_Rx_Data[10];      //
uint8_t   Up_Rx_Counter;      // 
uint8_t   Up_Rx_Max;          // 

uint8_t   Key_Rx_Data[20];  //接收数组
uint8_t   Key_Rx_Counter;
uint8_t   Key_Rx_Max;


uint8_t  IMUTGetFlag;


uint8_t   Command_Tx_Max;	   //发送的最大字节个数；
uint8_t   Command_Rx_Max;	   //接受的最大字节个数；

uint8_t   Comm_Sort;		       //命令类型
uint8_t   Data_Com_Flag;      //1--数据，2--命令

uint8_t   AHRSComFlag;		     //
uint8_t   COM_Flag;		       //串口发送标志位

//-----------------------------------------------------------------
// "EncryptID.h"
//-----------------------------------------------------------------
uint8_t   CPU_ID_Data[25];

//-----------------------------------------------------------------
//"stmflash.h"
//-----------------------------------------------------------------
uint8_t  Flash_Wing_Flag;     //Flash写入标志位
uint8_t  Flash_Wirte_Flag; 

uint8_t   Data_Flash_W_Flag;   //标志位----启动系统参数写入

uint8_t   Update_One;
uint8_t   Update_Two;

uint8_t   Update_Flash_W_Flag;
uint8_t   MAngle_Flash_W_Flag;

uint16_t  SVersionH;           // 软件版本号 年份
uint16_t  SVersionL;           // 软件版本号 月份
uint16_t  HVersionH;           // 硬件版本号 年份
uint16_t  HVersionL;           // 硬件版本号 月份
uint16_t  HVersionS;           // 硬件版本号 序号

uint8_t  Flash_Rdata[150];	   //切记，是存储数据个数的四倍！因为需要变换为uint32
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

IMU_DATA_T    IMUDataBuffer;	 //gty IMU所有指针均指向该地址....
GNSS_DATA_T   GNSSDataBuffer;  //gty GPS所有指针均指向该地址....

PIMU_DATA_T   pImuData;        //gty 指针IMU....
PGNSS_DATA_T  pGnssData;       //gty 指针GPS....

OUTPUT_INFO_T GINavResult;     //gty 组合导航输出的所有信息

GSAV_DATA_T   g_GsavInfo;       //Gty  获得GSA,GSV信息....

BOOL IMUDataReady;             //gty IMU数据是否准备好，如果准备好，每次用完，清零
BOOL GNSSDataReady;            //gty GNSS数据是否准备好，如果准备好，每次用完，清零

BOOL GNSSDataGetFlag;          //gty 用于数据同步
BOOL IMUDataGetFlag;           //gty 用于数据同步  

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
//测试PPS用-----------------------------------------
int time1;
int time0;

//测试低功耗用-----------------------------------
uint8_t Lowpow_Flag;
uint8_t Lowpow_Sleep;
int Choose_Lowpow_Flag=2;//1:Sleep mode  3:Standby mode
uint8_t Lowpow_Check;
char lowpower_timer_start;
char lowpower_start;
int Lowpow_time;
uint8_t lowpow;
 
//EEPROM测试使用----------------------------------
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

	Data_Ini();	             //必须在Data_Flash_Decode之前，保证IMU_Kind是根据Flash进行评判...
	Check_Lock_Code();
	
	delay_init(84); 
	Set_System();
	
	//---------------------------
	delay_ms(500);	
	ApluCode();              //加密....这个必须提前到Key_Flash_Decode之前...如果不加密，则不进行组合导航..
	
	MisAngle_Flash_Decode(); //
	Data_Flash_Decode();     //系统参数解码+数据保护

	//---------------------------
	USART2_BodRate=115200;
  USART_Configuration();

	
	if(User_Kind!=5)
	SendVersion();          // 发送版本..	
		
	Acc_Ini();              // 加速度计
	Gyr_Ini();              // 陀螺仪
	
	//BMI055A_Ini();		      //加速度计初始化
	//BMI055G_Ini();		      //陀螺仪初始化
	
	//BMI160测试初始化
	BMI160_Ini();	
	delay_ms(10);
		
	GINavInit();            //组合导航初始化...
		
	NVIC_Configuration();		// NVIC配置
	
 //------------------------------------------------
	GPS_Rate_Config();      // GPS  配置
	GpsConfig2();	
	
 //------------------------------------------------	
 //	IWDG_Init(4,2000);	    // 设置1S看门狗
	
	DIni_Flag=1;
	
	//低功耗-----------------------
	Interrupt_Any_motion();	//检测运动
	
	//EEPROM测试写--------------------------------
	 while( EEPROM_Check()!=0){};//检测EEROM连接成功
   EEPROM_Write(0,(u8*)TEXT_Buffer,SIZE);
		 
  while (1)
  {
		//睡眠模式测试------------------------------OK
		if(Lowpow_Sleep==1)//发送log sleep复位
		{
			  Lowpow_Sleep=0;
			  __set_FAULTMASK(1);      // 关闭所有中断
				NVIC_SystemReset();      // 复位	
		}
		if(Lowpow_Flag)//串口命令log lowpo进入低功耗
		{
			 Lowpow_Flag = 0;
			 //stop mode-----------------------模式二 PA0 160的INT1唤醒
			 if(Choose_Lowpow_Flag==2)
       {
				 
				 
				 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
				 PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI); 
				 __nop();
				 __nop();
		   } 
			 //Standby mode--------------------模式三
			 else if(Choose_Lowpow_Flag==3)
			 {   
					// GPIO_SetBits(GPIOA,GPIO_Pin_15);//关UBLOX
					 
					 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA  |RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOC,DISABLE); 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |RCC_APB1Periph_TIM3  |RCC_APB1Periph_TIM2 |RCC_APB1Periph_SPI2,DISABLE);
					 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG |RCC_APB2Periph_USART1 |RCC_APB2Periph_USART6|RCC_APB2Periph_SPI1,DISABLE);
					 
					 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
					 PWR_ClearFlag(PWR_FLAG_WU);//注：必须清除
					 PWR_WakeUpPinCmd(ENABLE);
					 PWR_EnterSTANDBYMode();
			 }
		 }	 
		 
		//160测试-------------------------------------OK
//		Get_BMI160G_Data();	//1-获得陀螺仪数据
//		Get_Rate_Data();    //1-获得基于陀螺仪的角度信息。
//		Get_Yaw_Data();     //1-获得基于陀螺仪的姿态数值；	

//		Get_BMI160A_Data();//2-获得加速度数据
//		Get_Acc_XYZ();	   //2-获得加速度数值；
//		Get_Angle_XYZ();   //2-获得基于加速度的姿态数值；	
		 
     //EEPROM-FM24C512测试-----------------------OK
//			EEPROM_Read(0,datatemp,SIZE);//eeprom测试
//			testcheck=EEPROM_ReadOneByte(65535);
//		  delay_ms(500);
		 
		 if(Up_On_Flag==0)
		 {	
			GINavProc(&GINavResult);     //组合导航处理....
			Pack_GnssData();             //整理GNSS数据....
			Send_GnssData();		         //发送GNSS数据....	
			Send_GSAData();              //获得数据...			 
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
	//  非常，非常重要！！！！！
	//****************************************************************************************
	DIni_Flag=0;
	//------------------------------------------------------
	//   一般用户  9600   1HZ   GGA,RMC GSA GSV
	//------------------------------------------------------
	//注意flash默认设置,这里的设置没有用！
	IMU_Kind     = 2;             //1: 天星北斗          2: 1216模块  X轴朝后...(天工MTK版本)  3:XXX            4：           5：1216模块 X轴朝前...  6:1点（超前+右上）澳门  7：Gmouse(创鑫电) 8：3反装（大唐）
	User_Kind    = 7;             //1: 普通客户          2：元征	                             3:华途           4:鹰瞰        5: 力浪                 6: 北京客户             7: 天工测试       8：星云互联      9:邵总

	Debug_Flag   = 0;	            //0：工作状态          1：调试状态  115200			

	GpsKind      = 1;             //1: Ulbox             2: Mtk
	ProductKind  = 1;             //1: 模块              2：Gmosue
	
	//****************************************************************************************
		
	Uart_Kind   = 2;              //1:Uart1->接收GPS Uart2->用户  2：反之
	
	Up_On_Flag  = 0;              //0: 导航协议                   1：调试协议	
	Gps_Fren_Kind  = 1;           //5: 默认5hz...一旦发现5HZ输入，则Sim_Mode=0....转换为根据GPS驱动.....
	//--------------------------------------
	//---------------------------------------
  
	Diff_First_Flag=0;
	
	BaseTime=10;                   //INS=10hz,BaseTime=20;  INS=20hz,BaseTime=10ms
	
	DetaTime=1000/5/BaseTime;     //阈值 默认55HZ输入，则5s->20......
	
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
	
	GPS_Send_OK     = 0;    //2--发送GPS数据完毕
	GNSS_Send_OK    = 0;    //3--发送GNSS数据完毕
	IMU_Send_OK     = 0;    //4--发送IMU数据完毕
	DMA_Send_Kind   = 0;    //Kind=1,则为GPS，Kind=2，则为IMU数据....
	
	
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



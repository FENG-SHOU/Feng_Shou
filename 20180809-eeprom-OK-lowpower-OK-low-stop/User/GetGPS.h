#ifndef  __GETGPS_H
#define  __GETGPS_H


#include "Config.h"
#include "stdint.h"

#define MAXFIELD	25
#define NP_MAX_CMD_LEN			8		  // maximum command length (NMEA address)
#define NP_MAX_DATA_LEN			180		// maximum data length
#define NP_ALL_DATA_LEN			900		// maximum data length
#define NP_IMU_DATA_LEN			300		// maximum data length
#define NP_MAX_CHAN				  36		// maximum number of channels
#define NP_WAYPOINT_ID_LEN	32		// waypoint max string len  
#define FALSE   0
#define TRUE    1

#define M              39940.64       // ???     ?? km
#define N              40075.86       // ???     ?? km



typedef enum  tag_GPS_CONFIG
{
	GPS_CONFIG_PRT_SET,
	GPS_CONFIG_GGA_OPEN,
	GPS_CONFIG_GST_OPEN,
	GPS_CONFIG_GSA_OPEN,
	GPS_CONFIG_GSV_OPEN,
	GPS_CONFIG_GSA_CLOSE,
	GPS_CONFIG_GSV_CLOSE,
	GPS_CONFIG_GLL_CLOSE,
	GPS_CONFIG_VTG_CLOSE,
	GPS_CONFIG_ZDA_OPEN,
	GPS_CONFIG_ZDA_CLOSE,
	GPS_CONFIG_RMC_OPEN,
	GPS_CONFIG_BD1_OPEN,
	GPS_CONFIG_BD2_OPEN,
	GPS_CONFIG_GL_OPEN,
	GPS_CONFIG_SAVE,
	GPS_CONFIG_RATE,
	
}GPS_CONFIG;


typedef	struct tag_GPGGA_DATA 
{ 
	uint8_t 	Hour;							//
	uint8_t 	Minute;							//
	uint8_t 	Second;							//
	uint16_t 	MSecond;					    //
	double 	  Latitude;						// < 0 = South, > 0 = North
	double 	  BLatitude;						// < 0 = South, > 0 = North
	uint8_t   SNth;
	double 	  Longitude;						// < 0 = West, > 0 = East
	double 	  BLongitude;						// < 0 = West, > 0 = East
	uint8_t   WEst;
	uint8_t 	GPSQuality;						// 0 = fix not available, 1 = GPS sps mode, 2 = Differential GPS, SPS mode, fix valid, 3 = GPS PPS mode, fix valid
	uint8_t 	NumOfSatsInUse;					//
	double 	  HDOP;							//
	double 	  Altitude;						// Altitude: mean-sea-level (geoid) meters
	uint8_t   Auint;
	double    Geoidal;                        // Geoid Separation
	uint8_t   Guint;
	double    DTime;                          // ���ʱ��
	uint16_t  DGpsID;                         // ���ID
	uint8_t   IDValid;
	uint32_t 	Count;							   //
	int32_t 	OldVSpeedSeconds;				//
	double 	  OldVSpeedAlt;					//
	double 	  VertSpeed;						//
  uint8_t   DFID_Data[4];
	uint8_t   Gp_PN_Kind;
	uint8_t   Utc_Flag;
} GPGGA_DATA;



/* GPRMC
Recommended Minimum Specific GPS/TRANSIT Data��RMC���Ƽ���λ��Ϣ
$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh<CR><LF>
<1> UTCʱ�䣬hhmmss��ʱ���룩��ʽ
<2> ��λ״̬��A=��Ч��λ��V=��Ч��λ
<3> γ��ddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<4> γ�Ȱ���N�������򣩻�S���ϰ���
<5> ����dddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<6> ���Ȱ���E����������W��������
<7> �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
<8> ���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
<9> UTC���ڣ�ddmmyy�������꣩��ʽ
<10> ��ƫ�ǣ�000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
<11> ��ƫ�Ƿ���E��������W������
<12> ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��
*/
typedef	struct tag_GPRMC_DATA { 
	uint8_t 	Hour;							//
	uint8_t 	Minute;							//
	uint8_t 	Second;							//
	uint16_t 	MSecond;							//
	uint8_t 	DataValid;						// A = Data valid, V = navigation rx warning
	double 	  Latitude;						// current latitude
	uint8_t   SNth;
	double 	  Longitude;						// current longitude
	uint8_t   WEst;
	double 	  GroundSpeed;					// speed over ground, knots
	double 	  Course;							// course over ground, degrees true
	uint8_t   SpeedValid;
	uint8_t   CourseValid;
	uint8_t 	Day;							//
	uint8_t 	Month;							//
	uint16_t 	Year;							//
	double 	  MagVar;							// magnitic variation, degrees East(+)/West(-)
	uint8_t   MagVarValid;
	uint8_t   MagWEst;                        // ModeIndicator A=Autonomous,D=Differential,E=Dead Reckoning,N=None;
	uint8_t   ModeIn;                         // ModeIndicator A=Autonomous,D=Differential,E=Dead Reckoning,N=None;
	uint32_t 	Count;							//
	uint8_t    Gp_PN_Kind;
} GPRMC_DATA; 




typedef	struct tag_GPVTG_DATA { 
   double		  Ttrack;
	 uint8_t    Tax;
   double		  Mtrack;
	 uint8_t    Max;
	 double		  speedNot;
	 uint8_t    NotMode;  
	 double		  speedkm;
	 uint8_t    KmMode;  
	 uint8_t    PositMode;  
	 uint8_t    CourseValid;
	 uint8_t    Gp_PN_Kind;
	 uint32_t   Count;
}GPVTG_DATA; 

typedef	struct tag_GPZDA_DATA { 
  uint8_t 	Hour;							//
	uint8_t 	Minute;						//
	uint8_t 	Second;						//
	uint16_t 	MSecond;					//
	
	uint8_t 	Day;							//
	uint8_t 	Month;						//
	uint16_t 	Year;							//
	
	uint8_t 	ZHour;							//
	uint8_t 	ZMinute;						//
	uint8_t   GpZDA_PN_Kind;
	
}GPZDA_DATA; 



typedef	struct tag_GPGST_DATA { 
	float   GstRms;
	float   GstDetaLat;
	float   GstDetaLon;
	float   GstDetaAli;
} GPGST_DATA;


/*GPGSA
GPS DOP and Active Satellites��GSA����ǰ������Ϣ
$GPGSA,<1>,<2>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<3>,<4>,<5>,<6>*hh<CR><LF>
<1> ģʽ��M=�ֶ���A=�Զ�
<2> ��λ���ͣ�1=û�ж�λ��2=2D��λ��3=3D��λ
<3> PRN�루α��������룩���������ڽ���λ�õ����Ǻţ�01~32��ǰ���0Ҳ�������䣩��
<4> PDOPλ�þ������ӣ�0.5~99.9��
<5> HDOPˮƽ�������ӣ�0.5~99.9��
<6> VDOP��ֱ�������ӣ�0.5~99.9��
*/
typedef	struct tag_GPGSA_DATA { 
	uint8_t 	Mode;						// M = manual, A = automatic 2D/3D
	uint8_t 	FixMode;				// 1 = fix not available, 2 = 2D, 3 = 3D
	uint32_t 	SatUse[MAX_SVID/ 32 + 1]; 	// ID of sats in solution
	double 	  PDOP;							//
	double   	HDOP;							//
	double 	  VDOP;							//
} GPGSA_DATA;


typedef	struct tag_GPGSV_DATA { 
	uint8_t   SatInViewNum;
	uint32_t 	SatInView[MAX_SVID/ 32 + 1]; // ID of sats in solution
	uint8_t   SatEl[MAX_SVID];		          //elevation
	uint8_t   SatAz[MAX_SVID];		          //azimuth   ����Ҳ����ν�����øñ���	
	uint8_t   SatCn0[MAX_SVID];		        //cn0       0-99
} GPGSV_DATA;





typedef enum tag_NP_STATE {
	NP_STATE_SOM =				0,		// Search for start of message
	NP_STATE_CMD,						// Get command
	NP_STATE_DATA,						// Get data
	NP_STATE_CHECKSUM_1,				// Get first checksum character
	NP_STATE_CHECKSUM_2,				// get second checksum character
} NP_STATE;


//-----------------------------------------------------------
extern uint8_t   GGA_Rx_Data[20];  //��������
extern uint8_t   GGA_Rx_Counter;
extern uint8_t   GGA_Rx_Max;

extern  uint8_t   m_GSPData_First;
extern  uint16_t  m_GSPData_Num;                    //
extern  uint16_t  m_GSPData_NumB;

extern  uint8_t   Gps_No_Num;
extern  uint16_t  Gps_No_Data_Num;
extern  uint8_t   Gps_No_Flag;


extern  uint16_t   GGARMC_Length;
extern  uint16_t   GGARMCGST_Length;

extern  uint8_t    GGA_Zero_Flag;
extern  uint8_t    RMC_Back_Flag;
//-----------------------------------------------------------
extern  uint8_t   m_GSPData[NP_ALL_DATA_LEN];       // NMEA data     NMEA??
extern  uint8_t   m_GSPDataB[NP_ALL_DATA_LEN];      // NMEA data     900  GSA+GSV+GST+GSV

extern  uint8_t   m_GPSpData[NP_MAX_DATA_LEN];       // NMEA data     NMEA??
extern  uint8_t   m_RMCpData[NP_MAX_DATA_LEN];       // NMEA data     NMEA����
	
extern uint8_t     GGA_Get_Flag;        //Gps????
extern uint8_t     RMC_Get_Flag;        //Gps????
extern uint8_t     VTG_Get_Flag;        //Gps????
extern uint8_t     GST_Get_Flag;        //Gps????
extern uint8_t     GSA_Get_Flag;        //
extern uint8_t     GSV_Get_Flag;        //

extern GPGGA_DATA  GPGGAData;			  //???,??????????
extern GPRMC_DATA  GPRMCData;
extern GPVTG_DATA  GPVTGData;
extern GPGST_DATA  GPGSTData;
extern GPGSA_DATA  GPGSAData;
extern GPGSV_DATA  GPGSVData;
extern GPZDA_DATA  GPZDAData;

extern  uint8_t    GetAgeDataFlag;   //
extern  uint16_t   GetAgeData;       //


extern uint8_t     Diff_First_Flag;
extern uint8_t    InPutDiffNum;     //


extern uint8_t     Gps_Fren_Kind;

extern uint8_t     Rmc_Vtg_Flag;

//-----------------------------------------------------------
extern  uint8_t    GGA_Ini_Flag;
extern  uint8_t    Base_No_Num;
extern  uint16_t   GSA_Length;


//-----------------------------------------------------------

extern uint8_t GGA_Config_flag;
extern uint8_t GST_Config_flag;
extern uint8_t GSA_Config_flag;
extern uint8_t GSV_Config_flag;
extern uint8_t GLL_Config_flag;
extern uint8_t VTG_Config_flag;
extern uint8_t ZDA_Config_flag;
extern uint8_t RMC_Config_flag;

extern uint8_t BD1_Config_flag;
extern uint8_t BD2_Config_flag;
extern uint8_t GL_Config_flag;
extern uint8_t RATE_Config_flag;
extern uint8_t PRT_Config_flag;
extern uint8_t PRT_Config_flag;
extern uint8_t timer_start_flag;
extern uint16_t ttime;

//-----------------------------------------------------------


void  GpsConfig2(void);
void  GPS_Rate_Config(void);
void  SendConfigMessage(GPS_CONFIG get_state);
void  Analyse_GpsData(void);
void  Gps_Receive_Handle(void);

uint8_t ProcessCommand(uint8_t *pCommand);
uint8_t DataJudge(uint8_t *pFieldData);
uint8_t GetField(uint8_t *pData, uint8_t *pField, int32_t nFieldNum, int32_t nMaxFieldLen);

void GpsBDOpen(void);
void GpsGLOpen(void);
void ProcessNMEA(uint8_t btData);
void ProcessGPGGA(uint8_t *pData);
void ProcessGPGST(uint8_t *pData);
void ProcessGPVTG(uint8_t *pData);
void ProcessGPRMC(uint8_t *pData);
void ProcessGPGSA(uint8_t *pData);
void ProcessGPGSV(uint8_t *pData);
void ProcessGPZDA(uint8_t *pData);

void HandleGpsData(uint8_t GpsData);
void HandleGpsIni(void);
void HandleGGAOn(unsigned char Comamnd);
void SaveGpsData(uint8_t btData);
void Send_GPSData(void);

void Get_RMC_Data(void);
void Get_GGA_RMC_GST(void);
void Get_GSA_GAV(void);
void GpsBautConfig(void);
void GpsGsvOpen(void);
void GpsGsvClose(void);
void Send_Term_Data(void);

#endif




#ifndef __GINAV_H__
#define __GINAV_H__
#include "DataTypes.h"
#include "stdint.h"


#ifdef __cplusplus
EXTERN "C"{
#endif

  
extern  uint8_t     Sys_Freq;	
extern  uint8_t   	GpsKind;
	
extern  uint8_t   	GpsInsGetFlag;   //
extern  uint8_t    	Debug_Flag;
extern  uint8_t    	User_Kind;
   
extern  uint8_t     High_DGet_Flag;	
extern  uint8_t     INS_DGet_Flag;  
extern  uint8_t     Gnss_Get_Flag;
extern  uint8_t     SGPS_DGet_Flag;       //Gps获得标志
	
extern 	uint8_t     ANG_Kind_Flag;        //	
extern 	uint8_t     ANG_Lock_Flag;        //是否寻找
extern 	uint8_t     ANG_DGet_Flag;        //是否寻找		
extern 	uint8_t     ANG_FGet_Flag;        //是否寻找
extern  int16_t     MisAngleData;         //安装角度	
	
extern FLOAT32  YZ_Acce_Deta;
extern FLOAT32  YZ_Dece_Deta;

extern FLOAT32  YZ_Turn_Deta;
extern FLOAT32  YZ_Turn_Anlge;

extern FLOAT32  YZ_Lane_Deta;
extern FLOAT32  YZ_Lane_Anlge;

extern FLOAT32  YZ_Coll_Deta;

extern FLOAT32  YZ_Stab_Deta;
extern FLOAT32  YZ_Stab_Time;

extern FLOAT32  YZ_Att_Deta;

extern FLOAT32  YZ_Att_Min_Deta;
extern FLOAT32  YZ_Att_Max_Deta;

	
extern 	uint8_t     RSE_DGet_Flag;        //是否寻找	
extern  uint8_t     RSE_FGet_Flag;        //重新搜索角度
extern  uint8_t     RSE_SGet_Flag;    

extern  POS_T       PositionB;            //经度

extern int16_t      Ini0_RollBack;        //
extern int16_t      Ini0_PitchBack;       //
extern int16_t      Ini0_RollNew;         //
extern int16_t      Ini0_PitchNew;        //
extern uint8_t      Ini0_KindBack;        //
extern uint8_t      Ini0_KindNew;         //


extern  uint8_t     MAngle_Flash_W_Flag;  //
extern  uint8_t     Up_On_Flag;           // 	
	
	
void   SetGNSSData(PGNSS_DATA_T pGNSSData);
void   GINavInit(void);
BOOL   GINavProc(POUTPUT_INFO_T pNavResult);

#ifdef __cplusplus
};
#endif
#endif

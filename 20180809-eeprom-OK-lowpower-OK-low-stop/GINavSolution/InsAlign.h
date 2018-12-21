#ifndef __INSALIGN_H__
#define __INSALIGN_H__

#include "DataTypes.h"
#include "DataProc.h"
#include "Config.h"

#define GYRO_BIAS_SMOOTHOR_LEN		10*INS_UPDATE_RATE
#define STATIC_ALIGN_THD			10
#define STATIC_ALIGN_SMOOTHOR_LEN	(1*INS_UPDATE_RATE)
#define MAX_ATT_MAINTAIN_MS			100000
#define ALIGN_HEADING_VEL_THD		4.0
#define START_CONFIRM_HEADING_VEL_THD		4.5
#define END_CONFIRM_HEADING_VEL_THD			5.0

extern uint8_t   INS_DGet_Flag;
extern int16_t    MisAngleData;         //安装角度
extern uint8_t   RSE_DGet_Flag;
extern double    Lat_Back;             //经度
extern double    Lon_Back;             //维度

extern uint8_t    POS_First_Flag;       //位置....
extern uint8_t    ANG_DGet_Flag;        //是否寻找	
extern uint8_t    Ini0_KindBack;        //
extern int16_t    Ini0_RollBack;        //
extern int16_t    Ini0_PitchBack;       //

extern int16_t   Pitch_Back;           //俯仰角
extern int16_t   Roll_Back;            //横滚角
extern int16_t   Head_Back;            //方向角

extern 

void InitGyroBias(PIMU_DATA_T pImuData, PGNSS_DATA_T pGnssData);
void INSAlign(PIMU_DATA_T pImuData, PGNSS_DATA_T pGnssData);
void ConfirmHeading(PGNSS_DATA_T pGnssData);

#endif

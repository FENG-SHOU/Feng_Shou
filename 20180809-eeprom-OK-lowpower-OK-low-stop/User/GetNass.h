#ifndef __GETNASS_H__
#define __GETNASS_H__

#include "stdint.h"
#include "DataProc.h"
#include "GetGPS.h"

#include "math.h"

//--------------------------------------------------


extern uint8_t    GpsDriverImuFlag;        //Gps????
extern uint8_t    GpsDriverAnalyseFlag;    //Gps????..
extern uint8_t    GpsDriverSendFlag;       //Gps????....

extern uint8_t    GpsInsGetFlag;           //GpsIns??¦Ì?

extern uint8_t    ZeroSecondFlag;
extern uint16_t   T3times;

extern uint8_t    Gnss_Get_Flag;
extern uint8_t    Gnss_Up_Flag;

//--------------------------------------------------
extern uint8_t   FrameFirst;
extern uint8_t   DetaTime;        //

extern uint8_t   Ins_Go_Flag;  

void  Analyse_GpsData(void);
void GetDiffAgeData(void);


#endif

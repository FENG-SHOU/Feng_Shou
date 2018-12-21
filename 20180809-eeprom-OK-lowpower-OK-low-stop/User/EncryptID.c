#include  "EncryptID.h"
#include  "GetIMU.h"

//--------------------------
//void Check_Lock_Code(void)
//--------------------------
void Check_Lock_Code(void)
{
uint8_t i=0;
	
/****获取ID******/

	  CPU_Get_ID[0]=*(vu32*)(ID_BaseAddress0);
    CPU_Get_ID[1]=*(vu32*)(ID_BaseAddress1);
    CPU_Get_ID[2]=*(vu32*)(ID_BaseAddress2);

/****加密算法，重新编译ID后再识别****/

	  CPU_ID[0] = CPU_Get_ID[0];
	  CPU_ID[1] = CPU_Get_ID[0]>>16;

	  CPU_ID[2] = CPU_Get_ID[1];	  
	  CPU_ID[3] = CPU_Get_ID[1]>>16;
	  
	  CPU_ID[4] = CPU_Get_ID[2];
	  CPU_ID[5] = CPU_Get_ID[2]>>16;
	
	
		 for(i=0;i<6;i++)
   {	
	  Change_SensorData(CPU_ID[i]);
		
	  CPU_ID_Data[0+i*4]  = Change_Data[0];
	  CPU_ID_Data[1+i*4]  = Change_Data[1];
	  CPU_ID_Data[2+i*4]  = Change_Data[2];
	  CPU_ID_Data[3+i*4]  = Change_Data[3];
   }
	  
}

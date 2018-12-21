/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
	



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stmflash.h"
#include "hw_config.h"
#include "GetGPS.h"
#include "SendGPS.h"
#include "GetIMU.h"
#include "delay.h"
#include "stdio.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void Message_Ack_handle(uint8_t recv);

uint8_t   Message_Rx_Data[12]; //配置信息接收数组
uint8_t   Message_Rx_Counter;
uint8_t   Message_Rx_Max;

 
void USART2_IRQHandler() 
{ 
	static uint8_t  uart2_data;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{    
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);   
		uart2_data   = USART_ReceiveData(USART2);
 
		HandleGpsData(uart2_data);		
    Message_Ack_handle(uart2_data);
  }
}



void USART1_IRQHandler() 
{ 
	static uint8_t  uart1_data;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{    
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		
		uart1_data    = USART_ReceiveData(USART1);

	  HandleUserData(uart1_data);	
  }
}


void USART6_IRQHandler() 
{ 
	static uint8_t  uart6_data;
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{    
		USART_ClearITPendingBit(USART6,USART_IT_RXNE); 
		
		uart6_data    = USART_ReceiveData(USART6);

	
  }
}


//-----------------------------
//void DMA_Finish(void)
//-----------------------------
void DMA_Finish(void)
{
	//---------------------------------------------------
	//            GPS
	//---------------------------------------------------	
	if(DMA_Send_Kind==1)	          //如果是GPS数据发送过程.....
  {
   GPS_Send_OK = 1;               //Gps数据发送完毕，可以发送IMU数据
		
	 if(Debug_Flag)                 //调试状态下，发生完GPS原始数据，在发生该数据....
	 {
	  PackDebug_Flag=1;	            //可以发送ATT+GSV.....
	 }	
  }
	//---------------------------------------------------
	//            IMU
	//---------------------------------------------------
	else 	if(DMA_Send_Kind==2)	   //如果是IMU数据发送过程.....
	{
   IMU_Send_OK  = 1;	
  }  
	//---------------------------------------------------
	//            GNSS
	//---------------------------------------------------
	else if(DMA_Send_Kind==3)	   //如果是IMU数据发送过程.....
	{
   GNSS_Send_OK     = 1;	
   GNSS_Send_OK_Flag =0;		
  }
	//---------------------------------------------------
	//            GNSS
	//---------------------------------------------------	
	else if(DMA_Send_Kind==4)	   //如果是IMU数据发送过程..... 
	{
   GSA_Send_OK = 1;            //GSA数据发送完毕，可以发送IMU数据
  }
	//---------------------------------------------------
	//            GNSS
	//---------------------------------------------------	
	else
	{
   ATT_Send_OK = 1;               //GSA数据发送完毕，可以发送IMU数据
  }
 //----------------------------------------------------	
}

void DMA2_Stream7_IRQHandler(void){
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)
	{ 
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		
		DMA_Cmd(DMA2_Stream7, DISABLE); 
		
	
	 //----------------------------------------------------
   DMA_Finish();		
	 
	//----------------------------------------------------	
	}
}

extern char lowpower_timer_start;
extern char lowpower_start;
extern int Lowpow_time;

void TIM3_IRQHandler(void)
{
	static uint8_t Dog_Num=0;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
	
		T1times++;
	  T2times++;
	
		if(timer_start_flag)
			ttime++;
		//低功耗测试用
		//if(lowpower_timer_start)
//			Lowpow_time++;
//		
//		if(Lowpow_time == 8000)
//			Lowpow_Flag = 1;
		
		switch (T1times)
		{
			case 1:
			IMU_Flag=1;
			break;
			
		  case 2:		
				
			if(Up_On_Flag!=0)
		  {
		 	 Dog_Num++;
			 if(Dog_Num>=20)
			 {
				Dog_Num=0;
			  IWDG_ReloadCounter();       //喂狗...		
			 }
		  }			
			
			break;	
			
			case 5:
	    Flash_Wirte_Flag=1;              //启动flash写入
			break;
			
			case 10:
			T1times=0;
			break;

			default:
			break;
		}
		
	  Gps_Receive_Handle();     //5--GPS数据解析		
		
		Handle_Ini();             //6--Gps初始化.....
			
	
}



void TIM2_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 
	
	if(DIni_Flag)
	{
	Get_BMI055_Data();
	
	Send_IMUData();  
	
	Pack_IMUData();
	
	Data_Synch();     

	Analyse_GpsData();           //解析GPS数据.......解析GGA+RMC/VTG+GST，进入GPSINS...	必须放在主函数，否则会重复进入GPSINS
	
	Send_GPSData();   
	
	Pack_Send_Debug();        //5-ATT+GSV...
	
	}
}



//----------------------------------
//void Handle_Ini(void)
//----------------------------------
void Handle_Ini(void)
{		 
	if(GGA_Ini_Flag)
	{
		GGA_Ini_Flag=0;
		

		 GpsDriverAnalyseFlag =0;   //??   2  
		 GpsDriverImuFlag     =0;   //??   3-1
	   GpsDriverSendFlag    =0;   //??   3-2
		

		 FrameFirst=1;        //   
	  
   	 GNSS_Send_OK  = 0;   //
     GPS_Send_OK   = 0;   // 
	 //IMU_Send_OK   = 0;   //   
		 ATT_Send_OK=0;
		
     T1times=0;           //   ???1  --Sn First
		 T2times=0;           //   ???2  --Sn First 

		 IMU_Num  =0;         //   IMU??     --Sn First 
		 SaveTime =0;         //   ????    --Sn First		
	   SendTime =0;         //   ????    --Sn First		
		 PackTime =0;	        //   ????    --Sn First	
		 
	   IMU_DMA_Num=0;    //   1HZ,5HZ,????.....			
		 

	 }		 
}


void Message_Ack_handle(uint8_t recv)
{
	 if(Message_Rx_Counter>=sizeof(Message_Rx_Data))
	 {
		Message_Rx_Counter=sizeof(Message_Rx_Data)-1;
	 }
	
   Message_Rx_Data[Message_Rx_Counter]=recv;
   Message_Rx_Counter++;
	
	if(Message_Rx_Counter<=4)
  {
	 //------------------------------------
		if(Message_Rx_Data[0] != 0XB5)
	  {	 
	   Message_Rx_Counter=0;			
	  }
	 //------------------------------------
	  if(Message_Rx_Counter==4)
	  {			
		 if((Message_Rx_Data[0]==0XB5)&&(Message_Rx_Data[1]==0x62)&&(Message_Rx_Data[3]==0x01)) 
     {
      Message_Rx_Max = 10;
     }
     else
	   {
      Message_Rx_Max = 1;
	    Message_Rx_Counter=0;
	   }		
		}
	 //------------------------------------
	}
	else
	{
	 if(Message_Rx_Counter>=Message_Rx_Max)
   {
	   Message_Rx_Counter=0;
   		 if((Message_Rx_Data[6]==0x06)&&(Message_Rx_Data[7]==0x01)) 
			 {
					GGA_Config_flag = 1;
					GST_Config_flag = 1;
				  GSA_Config_flag = 1;
					GSV_Config_flag = 1;
					GLL_Config_flag = 1;
					VTG_Config_flag = 1;
					ZDA_Config_flag = 1;
					RMC_Config_flag = 1;
			 }
			 
			 else if((Message_Rx_Data[6]==0x06)&&(Message_Rx_Data[7]==0x3E)) 
			 {
					BD1_Config_flag = 1;
				  GL_Config_flag = 1;
			 }
			 
			 else if((Message_Rx_Data[6]==0x06)&&(Message_Rx_Data[7]==0x17)) 
			 {
					BD2_Config_flag = 1;
			 }
			 
			 else if((Message_Rx_Data[6]==0x06)&&(Message_Rx_Data[7]==0x08)) 
			 {
					RATE_Config_flag = 1;
			 }
			 
			 else if((Message_Rx_Data[6]==0x06)&&(Message_Rx_Data[7]==0x00)) 
			 {
					PRT_Config_flag = 1;
			 }
			 
			 else
			 {
			 }			 
	 }
	}
}

extern int time1;
extern int time0;

void EXTI15_10_IRQHandler(void) //BMI160产生的外部中断，PA11，INT2，用于MCU睡眠模式唤醒
{
	if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
	{
	   SystemInit();
		 delay_ms(500);
		 NVIC_Configuration();		// NVIC配置
    EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void EXTI0_IRQHandler(void)//BMI160产生的外部中断，PA0，INT1，用于MCU待机模式唤醒
{
	//测试--------------------------------------------
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{
		__set_FAULTMASK(1);      // 关闭所有中断
		NVIC_SystemReset();      // 复位
    EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

//void EXTI15_10_IRQHandler(void)   //PPS产生中断，PB10
//{
//	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
//	{
//		USART_SendData(USART1,'\n');
//		
//		sprintf(ex_ch,"%d",ex_cent);
//		SerialPutString(ex_ch);
//		
//		USART_SendData(USART1,'\n');
//		
//		ex_cent++;
//		
//		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==1)
//		{
//			time1++;
//			GPIO_SetBits(GPIOA,GPIO_Pin_1);
//		}
//		else
//		{
//			time0++;
//			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//		}
//    EXTI_ClearITPendingBit(EXTI_Line10);
//	}
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

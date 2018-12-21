#include "stm32f4xx.h"
#include "hw_config.h"
#include "GetGyr.h"
#include "GetGps.h"
#include "SendGps.h"
#include "commdata.h"

#define I2C2_SLAVE_ADDRESS7     0x7A
#define ClockSpeed              20000


void Set_System(void)
{
	RCC_Configuration();      // 设置系统频率
	GPIO_Configuration();	    // 配置端口

	USART_Configuration();// 配置串口
  TIM3_Configuration();	    // TIM1配置
	USART_DMA_TX_Configuration(TDM_TX_Data); // 串口DMA 
 	SPI_Configuration();	    // 配置SPI口
	I2C_Configuration();	    // 配置I2C口
	EXTIX_Init();             // //INT 1

	
}

void RCC_Configuration(void)
{
	SystemInit();              //设置系统时钟为72MHz
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA  |RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |RCC_APB1Periph_TIM3  |RCC_APB1Periph_TIM2 |RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG |RCC_APB2Periph_USART1 |RCC_APB2Periph_USART6|RCC_APB2Periph_SPI1,ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	if(SOFT_I2C==0)
	{
	 RCC_AHB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	}

}

void GPIO_Configuration(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;

	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART1
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9,PA10
	
  //串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9,PA10
	
	  //串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10复用为USART1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9,PA10
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB13复用为SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB14复用为SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB15复用为SPI2
	
	//SPI2端口配置GPIOB13,14,15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PB13~15复用功能输出
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
  //低功耗Standby mode 端口配置GPIOA0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	//BMI160_CS端口配置GPIOA4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
 
 //PPS输出端口配置GPIOA1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	//pps产生端口配置PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	 //GPS使能A15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	//GPIO_SetBits(GPIOA,GPIO_Pin_15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	
}

void USART_Configuration()
{
	
  USART_InitTypeDef USART_InitStructure;
 
 if(Debug_Flag)
 {
	USART_InitStructure.USART_BaudRate = 115200;
 }
 else
 {
	if(Buat_DGet_Flag==2)  
	{
   USART_InitStructure.USART_BaudRate = 4800;         //Gmouse + 天星北斗...
	 High_DGet_Flag     = 0;
	}
	else if(Buat_DGet_Flag==1) 
	{
   USART_InitStructure.USART_BaudRate = 9600;         //Gmouse + 天星北斗...
	 High_DGet_Flag     = 0;
  }
	else if(Buat_DGet_Flag==3) 
	{
   USART_InitStructure.USART_BaudRate = 19200;         //Gmouse + 天星北斗...
	 High_DGet_Flag     = 0;
  }
	else if(Buat_DGet_Flag==4) 
	{
   USART_InitStructure.USART_BaudRate = 38400;         //Gmouse + 天星北斗...
	 High_DGet_Flag     = 0;
  }
	else
	{
   USART_InitStructure.USART_BaudRate = 115200;         //Gmouse + 天星北斗...
  }
 }
 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口2

	USART_Cmd(USART1, ENABLE);  //使能串口2 
  USART_ClearFlag(USART1, USART_FLAG_TC);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	
	if(Uart_Kind==2)
	{
	 USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);     //??USART2??DMA?? 
	}
	
	USART_InitStructure.USART_BaudRate = USART2_BodRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_Cmd(USART2, ENABLE);  //使能串口2 
  USART_ClearFlag(USART2, USART_FLAG_TC);	
	
	if(Uart_Kind==1)
	{
	 USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);     //??USART2??DMA?? 
	}
	
}

void USART_DMA_TX_Configuration(uint8_t *DMA_TTX_Data){

	DMA_InitTypeDef  DMA_InitStructure;
  DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t )DMA_TTX_Data;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = DMA_Tx_Num;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//初始化DMA Stream

	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
}



//通用定时器3中断初始化1ms
void TIM3_Configuration(void)
{
 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
  TIM_TimeBaseInitStructure.TIM_Period =9; 	//自动重装载值  1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=8399;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上技术模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	TIM_TimeBaseInitStructure.TIM_Period =9; 	//自动重装载值  0.1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=839;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上技术模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器2
	
	TIM_TimeBaseInitStructure.TIM_Period = 9; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=8399;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);//初始化TIM1	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //允许定时器2更新中断
  TIM_Cmd(TIM1,ENABLE); //使能定时器1
	
}

void SPI_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//

   SPI_Init(SPI2, &SPI_InitStructure);								  //????????????SPI2??????
   SPI_Cmd(SPI2, ENABLE); 											        //SPI2??	
}


void I2C_Configuration(void)
{
 if(SOFT_I2C)
 {
	IIC_Init(); 
 }
 else
 {	 
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
#if  USE_I2C1   //选用I2C1
   I2C_Init(I2C1, &I2C_InitStructure);
	 I2C_StretchClockCmd(I2C1, ENABLE);
	 I2C_Cmd(I2C1, ENABLE);
#else           //选用I2C2
  I2C_Init(I2C2, &I2C_InitStructure);
	I2C_StretchClockCmd(I2C2, ENABLE);
	I2C_Cmd(I2C2, ENABLE);
#endif
	 
	I2C1->CR1 |= 1<<7;
 }
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

		
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_InitStructure. NVIC_IRQChannelSubPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;
	NVIC_InitStructure. NVIC_IRQChannelSubPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);
	
	//INT 2 PA11
	NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	//INT 1 PA0
	NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void NVIC_ConfigurationDis(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_InitStructure. NVIC_IRQChannelSubPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;
	NVIC_InitStructure. NVIC_IRQChannelSubPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	
	NVIC_Init(&NVIC_InitStructure);
}

 //-----------INT 1--------------------------------------------------------
void EXTIX_Init(void)
{ 
  EXTI_InitTypeDef   EXTI_InitStructure;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode =EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
	

}
//void EXTIX_Init(void)//PPS，PB10
//{
// 

//  
//  EXTI_InitTypeDef   EXTI_InitStructure;

//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource10);
//  
//  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
//  EXTI_InitStructure.EXTI_Mode =EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Rising_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);

//}
//------------------------------------------------------
//  底层相关
//------------------------------------------------------
void SendDriver(void)
{
	 if(Uart_Kind==1)
	 {		 

	 }
	 else
	 {		  
		DMA_Cmd(DMA2_Stream7, ENABLE); 
	 }
 }


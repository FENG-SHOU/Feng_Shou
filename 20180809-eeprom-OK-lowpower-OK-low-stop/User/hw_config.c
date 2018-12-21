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
	RCC_Configuration();      // ����ϵͳƵ��
	GPIO_Configuration();	    // ���ö˿�

	USART_Configuration();// ���ô���
  TIM3_Configuration();	    // TIM1����
	USART_DMA_TX_Configuration(TDM_TX_Data); // ����DMA 
 	SPI_Configuration();	    // ����SPI��
	I2C_Configuration();	    // ����I2C��
	EXTIX_Init();             // //INT 1

	
}

void RCC_Configuration(void)
{
	SystemInit();              //����ϵͳʱ��Ϊ72MHz
	
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

	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10����ΪUSART1
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9,PA10
	
  //����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9,PA10
	
	  //����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10����ΪUSART1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA9,PA10
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB13����ΪSPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB14����ΪSPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB15����ΪSPI2
	
	//SPI2�˿�����GPIOB13,14,15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PB13~15���ù������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
  //�͹���Standby mode �˿�����GPIOA0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	//BMI160_CS�˿�����GPIOA4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
 
 //PPS����˿�����GPIOA1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	//pps�����˿�����PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	 //GPSʹ��A15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
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
   USART_InitStructure.USART_BaudRate = 4800;         //Gmouse + ���Ǳ���...
	 High_DGet_Flag     = 0;
	}
	else if(Buat_DGet_Flag==1) 
	{
   USART_InitStructure.USART_BaudRate = 9600;         //Gmouse + ���Ǳ���...
	 High_DGet_Flag     = 0;
  }
	else if(Buat_DGet_Flag==3) 
	{
   USART_InitStructure.USART_BaudRate = 19200;         //Gmouse + ���Ǳ���...
	 High_DGet_Flag     = 0;
  }
	else if(Buat_DGet_Flag==4) 
	{
   USART_InitStructure.USART_BaudRate = 38400;         //Gmouse + ���Ǳ���...
	 High_DGet_Flag     = 0;
  }
	else
	{
   USART_InitStructure.USART_BaudRate = 115200;         //Gmouse + ���Ǳ���...
  }
 }
 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������2

	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���2 
  USART_ClearFlag(USART1, USART_FLAG_TC);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	
	if(Uart_Kind==2)
	{
	 USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);     //??USART2??DMA?? 
	}
	
	USART_InitStructure.USART_BaudRate = USART2_BodRate;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 
  USART_ClearFlag(USART2, USART_FLAG_TC);	
	
	if(Uart_Kind==1)
	{
	 USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);     //??USART2??DMA?? 
	}
	
}

void USART_DMA_TX_Configuration(uint8_t *DMA_TTX_Data){

	DMA_InitTypeDef  DMA_InitStructure;
  DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t )DMA_TTX_Data;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = DMA_Tx_Num;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//��ʼ��DMA Stream

	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
}



//ͨ�ö�ʱ��3�жϳ�ʼ��1ms
void TIM3_Configuration(void)
{
 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
  TIM_TimeBaseInitStructure.TIM_Period =9; 	//�Զ���װ��ֵ  1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=8399;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	TIM_TimeBaseInitStructure.TIM_Period =9; 	//�Զ���װ��ֵ  0.1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=839;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	
	TIM_TimeBaseInitStructure.TIM_Period = 9; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=8399;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);//��ʼ��TIM1	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
  TIM_Cmd(TIM1,ENABLE); //ʹ�ܶ�ʱ��1
	
}

void SPI_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
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
#if  USE_I2C1   //ѡ��I2C1
   I2C_Init(I2C1, &I2C_InitStructure);
	 I2C_StretchClockCmd(I2C1, ENABLE);
	 I2C_Cmd(I2C1, ENABLE);
#else           //ѡ��I2C2
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
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

		
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
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
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
//void EXTIX_Init(void)//PPS��PB10
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
//  �ײ����
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


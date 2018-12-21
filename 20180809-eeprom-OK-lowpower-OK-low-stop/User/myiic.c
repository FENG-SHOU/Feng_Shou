#include "myiic.h"
#include "delay.h"

#include "stm32f4xx_i2c.h"

#define Dt              20

//��ʼ��IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	
	
  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	IIC_SCL=1;
	IIC_SDA=1;
	
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	
  delay_us(Dt);	//plus
	IIC_SCL=1;
	delay_us(Dt);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(Dt);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	delay_us(Dt);	//plus
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(Dt);
	IIC_SCL=1; 
	delay_us(Dt);	//plus
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(Dt);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(Dt);	   
	IIC_SCL=1;delay_us(Dt);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(Dt);
	IIC_SCL=1;
	delay_us(Dt);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(Dt);
	IIC_SCL=1;
	delay_us(Dt);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(Dt);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(Dt); 
		IIC_SCL=0;	
		delay_us(Dt);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(Dt);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}




/**
  *****************************************************************************
  * @Name   : Ӳ��IIC�ȴ����豸�ڲ��������
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:  ��ȡ��EEPROM�ڴ��ַ
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
void Hard_IICWaiteStandby(I2C_TypeDef* IICx, uint8_t SlaveAdd)
{
	u16 tmp = 0;
	
	IICx->SR1 &= 0x0000;  //���״̬�Ĵ���1
	
	do
	{
		I2C_GenerateSTART(IICx, ENABLE);  //������ʼ�ź�
		tmp = IICx->SR1;  //��ȡSR1�Ĵ�����Ȼ��д�����ݼĴ������������SBλ��EV5
		I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //���ʹ��豸��ַ
	} while ((IICx->SR1 & 0x0002) == 0x0000);  //��ADDR = 1ʱ������Ӧ���ˣ�����ѭ��
	I2C_ClearFlag(IICx, I2C_FLAG_AF);  //���Ӧ��ʧ�ܱ�־λ
	I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC����һ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           WriteAdd: д��EEPROM�ڴ��ַ
  *           Data:     д�������
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : none
  *****************************************************************************
**/
void Hard_IICWriteOneByte(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 WriteAdd, u8 Data, u8 * err)
{
	u16 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	
	I2C_GenerateSTART(IICx, ENABLE);  //������ʼ�ź�
	temp = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<1;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	temp = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		temp++;
		if (temp > 1000)
		{
			*err |= 1<<2;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	//
	//��ȡSR2״̬�Ĵ���
	//
	flag = IICx->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C_SendData(IICx, WriteAdd);  //���ʹ洢��ַ
	temp = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<3;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_SendData(IICx, Data);  //��������
	temp = 0;
	//
	//EV8_2
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<4;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC��ȡһ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:  ��ȡ��EEPROM�ڴ��ַ
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
u8 Hard_IIC_ReadOneByte(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 ReadAdd, u8 * err)
{
	u16 i = 0;
	u8 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //������ʼ�ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<1;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<2;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	flag = IICx->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C_SendData(IICx, ReadAdd);  //���ʹ洢��ַ
	i = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		i++;
		if (i > 2000)
		{
			*err |= 1<<3;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //�����ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<4;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Receiver);  //��ȡ����
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<5;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	flag = IICx->SR2;
	
	I2C_AcknowledgeConfig(IICx, DISABLE);  //����NACK
	I2C_GenerateSTOP(IICx, ENABLE);
	i = 0;
	//
	//EV7
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<6;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return 0;
		}
	}
	temp = I2C_ReceiveData(IICx);
	I2C_AcknowledgeConfig(IICx, ENABLE);
	
	return temp;
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC���Ͷ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:       IIC��
  *           SlaveAdd:   ��Ϊ���豸ʱʶ���ַ
  *           WriteAdd:   д��EEPROM�ڴ���ʼ��ַ
  *           NumToWrite: д��������
  *           *pBuffer:   д��������黺��
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : none
  *****************************************************************************
**/
void Hard_IIC_WriteNByte(I2C_TypeDef * IICx, u8 SlaveAdd, u8 WriteAdd, u8 NumToWrite, u8 * pBuffer, u8 * err)
{
	u16 sta = 0;
	u16 temp = 0;
	
	
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //������ʼ�ź�
	temp = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<1;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	temp = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		temp++;
		if (temp > 1000)
		{
			*err |= 1<<2;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	//
	//��ȡSR2״̬�Ĵ���
	//
	sta = IICx->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	I2C_SendData(IICx, WriteAdd);  //���ʹ洢��ַ
	temp = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<3;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	//
	//ѭ����������
	//
	while (NumToWrite--)
	{
		I2C_SendData(IICx, *pBuffer);  //��������
		pBuffer++;
		temp = 0;
		//
		//EV8_2
		//
		while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			temp++;
			if (temp > 800)
			{
				*err |= 1<<4;
				I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
				return;
			}
		}
	}
	I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC��ȡ����ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:      IIC��
  *           SlaveAdd:  ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:   ��ȡ��EEPROM�ڴ���ʼ��ַ
  *           NumToRead: ��ȡ����
  *
  * @Output : *pBuffer: �������������
  *           *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
void Hard_IIC_PageRead(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 ReadAdd, u8 NumToRead, u8 * pBuffer, u8 * err)
{
	u16 i = 0;
	u16 temp = 0;
	u16 sta = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //������ʼ�ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<1;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<2;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	sta = IICx->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C_SendData(IICx, ReadAdd);  //���ʹ洢��ַ
	i = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		i++;
		if (i > 2000)
		{
			*err |= 1<<3;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //�����ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<4;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Receiver);  //��ȡ����
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<5;
			I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
			return;
		}
	}
	sta = IICx->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	//
	//������������
	//
//	while (NumToRead--)
//	{
//		if (NumToRead == 1)  //���һ�������ˣ�������Ӧ���ź�
//		{
//			I2C_AcknowledgeConfig(IICx, DISABLE);  //����NACK
//			I2C_GenerateSTOP(IICx, ENABLE);
//		}
//		//
//		//�ж�RxNE�Ƿ�Ϊ1��EV7
//		//
//		i = 0;
//		while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_RECEIVED))
//		{
//			i++;
//			if (i > 1000)
//			{
//				*err |= 1<<6;
//				I2C_GenerateSTOP(IICx, ENABLE);  //����ֹͣ�ź�
//				return;
//			}
//		}
//		*pBuffer = I2C_ReceiveData(IICx);
//		pBuffer++;
//	}
	while (NumToRead)
	{
		if (NumToRead == 1)  //���һ�������ˣ�������Ӧ���ź�
		{
			I2C_AcknowledgeConfig(IICx, DISABLE);  //����NACK
			I2C_GenerateSTOP(IICx, ENABLE);
		}
		//
		//�ж�RxNE�Ƿ�Ϊ1��EV7
		//
		if (I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			*pBuffer = I2C_ReceiveData(IICx);
			pBuffer++;
			NumToRead--;
		}
	}
	I2C_AcknowledgeConfig(IICx, ENABLE);
}
























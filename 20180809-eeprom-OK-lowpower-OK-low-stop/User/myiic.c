#include "myiic.h"
#include "delay.h"

#include "stm32f4xx_i2c.h"

#define Dt              20

//初始化IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	
	
  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	IIC_SCL=1;
	IIC_SDA=1;
	
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	
  delay_us(Dt);	//plus
	IIC_SCL=1;
	delay_us(Dt);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(Dt);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	delay_us(Dt);	//plus
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(Dt);
	IIC_SCL=1; 
	delay_us(Dt);	//plus
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(Dt);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(Dt);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(Dt); 
		IIC_SCL=0;	
		delay_us(Dt);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}




/**
  *****************************************************************************
  * @Name   : 硬件IIC等待从设备内部操作完成
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC组
  *           SlaveAdd: 作为从设备时识别地址
  *           ReadAdd:  读取的EEPROM内存地址
  *
  * @Output : *err:     返回的错误值
  *
  * @Return : 读取到的数据
  *****************************************************************************
**/
void Hard_IICWaiteStandby(I2C_TypeDef* IICx, uint8_t SlaveAdd)
{
	u16 tmp = 0;
	
	IICx->SR1 &= 0x0000;  //清除状态寄存器1
	
	do
	{
		I2C_GenerateSTART(IICx, ENABLE);  //产生起始信号
		tmp = IICx->SR1;  //读取SR1寄存器，然后写入数据寄存器操作来清除SB位，EV5
		I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //发送从设备地址
	} while ((IICx->SR1 & 0x0002) == 0x0000);  //当ADDR = 1时，表明应答了，跳出循环
	I2C_ClearFlag(IICx, I2C_FLAG_AF);  //清除应答失败标志位
	I2C_GenerateSTOP(IICx, ENABLE);  //发送停止信号
}

/**
  *****************************************************************************
  * @Name   : 硬件IIC发送一个字节数据
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC组
  *           SlaveAdd: 作为从设备时识别地址
  *           WriteAdd: 写入EEPROM内存地址
  *           Data:     写入的数据
  *
  * @Output : *err:     返回的错误值
  *
  * @Return : none
  *****************************************************************************
**/
void Hard_IICWriteOneByte(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 WriteAdd, u8 Data, u8 * err)
{
	u16 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //等待IIC
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	
	I2C_GenerateSTART(IICx, ENABLE);  //产生起始信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //发送设备地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	//
	//读取SR2状态寄存器
	//
	flag = IICx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(IICx, WriteAdd);  //发送存储地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_SendData(IICx, Data);  //发送数据
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
}

/**
  *****************************************************************************
  * @Name   : 硬件IIC读取一个字节数据
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC组
  *           SlaveAdd: 作为从设备时识别地址
  *           ReadAdd:  读取的EEPROM内存地址
  *
  * @Output : *err:     返回的错误值
  *
  * @Return : 读取到的数据
  *****************************************************************************
**/
u8 Hard_IIC_ReadOneByte(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 ReadAdd, u8 * err)
{
	u16 i = 0;
	u8 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //等待IIC
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //发送起始信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //发送设备地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	flag = IICx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(IICx, ReadAdd);  //发送存储地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //重启信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Receiver);  //读取命令
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	flag = IICx->SR2;
	
	I2C_AcknowledgeConfig(IICx, DISABLE);  //发送NACK
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return 0;
		}
	}
	temp = I2C_ReceiveData(IICx);
	I2C_AcknowledgeConfig(IICx, ENABLE);
	
	return temp;
}

/**
  *****************************************************************************
  * @Name   : 硬件IIC发送多个字节数据
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:       IIC组
  *           SlaveAdd:   作为从设备时识别地址
  *           WriteAdd:   写入EEPROM内存起始地址
  *           NumToWrite: 写入数据量
  *           *pBuffer:   写入的数据组缓存
  *
  * @Output : *err:     返回的错误值
  *
  * @Return : none
  *****************************************************************************
**/
void Hard_IIC_WriteNByte(I2C_TypeDef * IICx, u8 SlaveAdd, u8 WriteAdd, u8 NumToWrite, u8 * pBuffer, u8 * err)
{
	u16 sta = 0;
	u16 temp = 0;
	
	
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //等待IIC
	{
		temp++;
		if (temp > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //产生起始信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //发送设备地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	//
	//读取SR2状态寄存器
	//
	sta = IICx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	I2C_SendData(IICx, WriteAdd);  //发送存储地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	//
	//循环发送数据
	//
	while (NumToWrite--)
	{
		I2C_SendData(IICx, *pBuffer);  //发送数据
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
				I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
				return;
			}
		}
	}
	I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
}

/**
  *****************************************************************************
  * @Name   : 硬件IIC读取多个字节数据
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:      IIC组
  *           SlaveAdd:  作为从设备时识别地址
  *           ReadAdd:   读取的EEPROM内存起始地址
  *           NumToRead: 读取数量
  *
  * @Output : *pBuffer: 数据输出缓冲区
  *           *err:     返回的错误值
  *
  * @Return : 读取到的数据
  *****************************************************************************
**/
void Hard_IIC_PageRead(I2C_TypeDef* IICx, uint8_t SlaveAdd, u8 ReadAdd, u8 NumToRead, u8 * pBuffer, u8 * err)
{
	u16 i = 0;
	u16 temp = 0;
	u16 sta = 0;
	
	while (I2C_GetFlagStatus(IICx, I2C_FLAG_BUSY))  //等待IIC
	{
		i++;
		if (i > 800)
		{
			*err |= 1<<0;
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //发送起始信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Transmitter);  //发送设备地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	sta = IICx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(IICx, ReadAdd);  //发送存储地址
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_GenerateSTART(IICx, ENABLE);  //重启信号
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	I2C_Send7bitAddress(IICx, SlaveAdd, I2C_Direction_Receiver);  //读取命令
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
			I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
			return;
		}
	}
	sta = IICx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	//
	//批量接收数据
	//
//	while (NumToRead--)
//	{
//		if (NumToRead == 1)  //最后一个数据了，不发送应答信号
//		{
//			I2C_AcknowledgeConfig(IICx, DISABLE);  //发送NACK
//			I2C_GenerateSTOP(IICx, ENABLE);
//		}
//		//
//		//判断RxNE是否为1，EV7
//		//
//		i = 0;
//		while (!I2C_CheckEvent(IICx, I2C_EVENT_MASTER_BYTE_RECEIVED))
//		{
//			i++;
//			if (i > 1000)
//			{
//				*err |= 1<<6;
//				I2C_GenerateSTOP(IICx, ENABLE);  //产生停止信号
//				return;
//			}
//		}
//		*pBuffer = I2C_ReceiveData(IICx);
//		pBuffer++;
//	}
	while (NumToRead)
	{
		if (NumToRead == 1)  //最后一个数据了，不发送应答信号
		{
			I2C_AcknowledgeConfig(IICx, DISABLE);  //发送NACK
			I2C_GenerateSTOP(IICx, ENABLE);
		}
		//
		//判断RxNE是否为1，EV7
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
























/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���		��i2c.c
 * ��	��		��STM32ͨ��ģ��IIC����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.2.150409
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾		��ST3.0.0
 * ����ʱ��	��2014.8.31
 * ���༭	��2015.4.9
 * ��			ע������FCU�ɿس����е�IIC�����޸ģ�����ʱ��,��Ұ�𿪷����в��Ժ�ʹ,
							V1.2�����»ָ���Ӳ����IIC ������IIC�ĳ�ʼ�������޸ģ�Ԥ���ò�����ʹ������������ **-------------------------------------------------------------------------------
**-------------------------------------------------------------------------------

 * ��	��		��Damm Stanger
 * ��	��		��dammstanger@qq.com
**********************************************************************************************/





#include "i2c.h" 
/***********************************ģ��IIC*********************************************/
#ifdef SOFT_I2C

void delay_us(u8 n)   // 72MHz�µ���׼ȷ
{ u16 a =n*10;	
 for(;a>0;a--);
    
}


static void I2C_delay(void)	//i=14 Լ3us
{
    volatile int i = 14; //40;  //
    while (i)
        i--;
}

/*
 * ��������I2C_GPIO_Config
 * ����  ��I2C1 I/O����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */

void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(IIC_GPIO_CLK, ENABLE );	
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;// ���� CLK �� SDA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;//   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(IIC_GPIO,IIC_SCL_PIN | IIC_SDA_PIN);
 
	IIC_SCL(1);//����ʱ����Ϊ��
	IIC_SDA(1);
}


//����IIC��ʼ�ź�
static bool IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	I2C_delay();
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	I2C_delay();
	IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
	return TRUE ;
}	  
//����IICֹͣ�ź�
static void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	IIC_SCL(1); 
	I2C_delay();
	IIC_SDA(1);//����I2C���߽����ź�
	I2C_delay();
}
//�ȴ�Ӧ���źŵ���
//����ֵ��FALSE������Ӧ��ʧ��
//        TRUE������Ӧ��ɹ�
static bool IIC_Wait_Ack(void)
{
	SDA_IN();      //SDA����Ϊ����  
	IIC_SCL(1);
	I2C_delay();
	if(READ_SDA)
	{
		IIC_SCL(0);
		I2C_delay();
        return FALSE;
	}
	IIC_SCL(0);//ʱ�����0 	 
	I2C_delay();	
	return TRUE;  
} 
//����ACKӦ��
static void IIC_Ack(void)
{
	SDA_OUT();
	IIC_SDA(0);
	IIC_SCL(1);
	I2C_delay();
	IIC_SCL(0);
	I2C_delay();
}
//������ACKӦ��		    
static void IIC_NAck(void)
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
    I2C_delay();
	IIC_SCL(0);
	I2C_delay();
}					 	

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
   	SDA_OUT(); 	    
    for(t=0;t<8;t++)
    {   
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 
		IIC_SCL(1);		
		I2C_delay();
		IIC_SCL(0);
		I2C_delay();
	}	 
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA(1);
	SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
        receive<<=1;
        IIC_SCL(1);
		I2C_delay();
		if(READ_SDA)receive |= 0x01;
		IIC_SCL(0);
		I2C_delay();      
	}
    if (ack)
		IIC_Ack(); //����ACK   
    else
		IIC_NAck();//����nACK 
    return receive;
}

bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data)
{	bool retval;
	IIC_Start();                   	//��ʼ�ź�
	IIC_Send_Byte(slaveAddr);   	//�����豸��ַ+д�ź�
	retval=IIC_Wait_Ack();	   
	IIC_Send_Byte(REG_Address);    	//�ڲ��Ĵ�����ַ
		IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(REG_data);       	//�ڲ��Ĵ�������
		IIC_Wait_Ack(); 	 											  		   
	IIC_Stop();                    	//����ֹͣ�ź�
	return retval;
}


u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr)
{
	u8 REG_data;
	IIC_Start();                   	//��ʼ�ź�
	IIC_Send_Byte(slaveAddr);    	//�����豸��ַ+д�ź�
	IIC_Wait_Ack();
	IIC_Send_Byte(readAddr);     	//���ʹ洢��Ԫ��ַ����0��ʼ
	IIC_Wait_Ack();					//����Ҫ��
	IIC_Start();                   	//��ʼ�ź�
	IIC_Send_Byte(slaveAddr+1);  	//�����豸��ַ+���ź�
	IIC_Wait_Ack();
	REG_data=IIC_Read_Byte(0);    	//�����Ĵ������ݲ������ٶ�,����NACK  
	return REG_data;	
}

u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	IIC_Start();                   //��ʼ�ź� 				 
	IIC_Send_Byte(slaveAddr);				//����д����ָ��	 
	if(FALSE==IIC_Wait_Ack())
	{
		IIC_Stop();                    		//����ֹͣ�ź�
		return 1;
	}	IIC_Send_Byte(readAddr);   			//���ͼĴ�����ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Start();  	 	   					//��������
	IIC_Send_Byte(slaveAddr+1);				//���Ͷ�����ָ��
	IIC_Wait_Ack();
	/* While there is data to be read */
	while(NumByteToRead)
	{
		if(NumByteToRead == 1)
		{
			*pBuffer=IIC_Read_Byte(0);    	//�����Ĵ������ݲ������ٶ�,����NACK  
		}
		else 
		{
			*pBuffer=IIC_Read_Byte(1);    	//�����Ĵ������ݼ�����,����ACK  
			pBuffer++;						/* Point to the next location where the byte read will be saved */
		}
		/* Decrement the read bytes counter */
		NumByteToRead--;
	}
	IIC_Stop();                    			//ֹͣ�ź�
	return 0;
}

u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite)
{
	IIC_Start();					  		//��ʼ�ź�

	IIC_Send_Byte(slaveAddr);   			//�����豸��ַ+д�ź�
		if(FALSE==IIC_Wait_Ack())
		{
			IIC_Stop();                    	//����ֹͣ�ź�
			return 1;
		}
	IIC_Send_Byte(wrtieAddr);				//�ڲ��Ĵ�����ַ
		IIC_Wait_Ack();
	while(NumByteToWrite)
	{
		IIC_Send_Byte(*pBuffer);       		//�ڲ��Ĵ�������		
		if(FALSE==IIC_Wait_Ack())
		{
			IIC_Stop();						//����ֹͣ�ź�
			return 1;
		}
		pBuffer++;
		NumByteToWrite--;
	} 										  		   
	IIC_Stop();                    			//����ֹͣ�ź�
	return 0;
}


#else
/***********************************Ӳ��IIC*********************************************/


/*
 * ��������I2C_GPIO_Config
 * ����  ��I2C1 I/O����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */

void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ���� I2C1 �йص�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);//����IOʱ�ӿ���

	/*IIC remapIO config*/
	GPIO_PinRemapConfig(GPIO_Remap_I2C1 , ENABLE);
	
	/* PB8-I2C1_SCL��PB9-I2C1_SDA*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ���ÿ�©��� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
 * ��������I2C_Configuration
 * ����  ��I2C ����ģʽ����
 * ����  ��uint8_t ownaddr, uint16_t I2Cspeed
 * ���  ����
 * ����  ���ڲ�����
 */
void I2C_Mode_Config(uint8_t ownaddr, uint32_t I2Cspeed)
{
	I2C_InitTypeDef  I2C_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  

	I2C_DeInit(I2C1);																		//��λ���裬һ��Ҫ��
	/* I2C ���� */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = ownaddr;			//��STM32��Ϊ�ӻ�ʱ��Ӧ���ַ��������ʱ�������
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2Cspeed;

	/* ʹ�� I2C1 */
	I2C_Cmd(I2C1, ENABLE);			//PE = 1;
	
	/* I2C1 ��ʼ�� */
	I2C_Init(I2C1, &I2C_InitStructure);

//	/*����1�ֽ�1Ӧ��ģʽ*/
//	I2C_AcknowledgeConfig(I2C1, ENABLE);    
}

/***************************************************************************
 * ��������IIC1_WriteByte
 * ����  ����I2C�豸д��һ���ֽ�����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
***************************************************************************/

bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data)
{
	bool retval;
	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	 I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	  
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, REG_Address);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(I2C1, REG_data); 

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);	
	
	return retval;
}


/***************************************************************************
 * ��������IIC1_MultWrite
 * ����  ����I2C�豸д�����ֽ�����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
***************************************************************************/
u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite)
{
	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	 I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	  
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, wrtieAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	 /* While there is data to be written */
  while(NumByteToWrite--)  
  {
    /* Send the current byte */
    I2C_SendData(I2C1, *pBuffer); 

    /* Point to the next byte to be written */
    pBuffer++; 
  
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);		
}

/***************************************************************************
 * ��������IIC1_ReadByte
 * ����  ��IIC1 ����һ�ֽ�
 * ����  ��
 * ���  ��
 * ����  ���ⲿ����
***************************************************************************/

u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr)
{
	u8 readval = 0x55;
	//*((u8 *)0x4001080c) |=0x80; 
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008


	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	//*((u8 *)0x4001080c) &=~0x80;

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);							

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, readAddr);  

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);


	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	
	/* Send STOP Condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	  
	/* Test on EV7 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); 
	    
	/* Read a byte from the EEPROM */
	readval = I2C_ReceiveData(I2C1);   
	
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return readval;
}

/***************************************************************************
 * ��������
 * ����  ��
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
***************************************************************************/

u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
 // ENTR_CRT_SECTION();

  /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter); 

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2C1, readAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
//  EXT_CRT_SECTION();
return 0;
}

#endif



void My_I2C_Init(uint8_t ownaddr, uint32_t I2Cspeed)
{
	I2C_GPIO_Config();
#ifdef	HARDWARE_I2C
	I2C_Mode_Config(ownaddr,I2Cspeed);
#endif
}







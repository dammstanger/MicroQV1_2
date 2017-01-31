/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���		��i2c.h
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
#ifndef _I2C_H_
#define	_I2C_H_


#include "Project_cfg.h"
#include "stm32f10x.h"


//ģ��IICӲ��IIC
#define SOFT_I2C
//#define HARDWARE_I2C

//IIC ����
#define I2C_Speed              400000											//����IIC����Ϊ400K

#ifdef SOFT_I2C
/******************************************************/


//IO
#define IIC_GPIO_CLK RCC_APB2Periph_GPIOB 
#define IIC_GPIO     GPIOB 
#define IIC_SCL_PIN  GPIO_Pin_6  //����
#define IIC_SDA_PIN  GPIO_Pin_7  //


//IO��������
#define SDA_IN()  {IIC_GPIO->BSRR|=IIC_SDA_PIN;IIC_GPIO->CRH&=0XFFFFFF0F;IIC_GPIO->CRH|=8<<4;} 	//����������ģʽ
#define SDA_OUT() {IIC_GPIO->CRH&=0XFFFFFF0F;IIC_GPIO->CRH|=5<<4;} 								//�ڿ�©���ģʽ �ٶ�10MHz

#define IIC_SCL(a)   if (a)  \
				 	 IIC_GPIO->BSRR|=IIC_SCL_PIN;else IIC_GPIO->BRR|=IIC_SCL_PIN //�����ģʽ(MODE[1:0]>00)�� 
#define IIC_SDA(a)   if (a)  \
				 	 IIC_GPIO->BSRR|=IIC_SDA_PIN;else IIC_GPIO->BRR|=IIC_SDA_PIN //�����ģʽ(MODE[1:0]>00)�� 

#define READ_SDA	 GPIOB->IDR  & IIC_SDA_PIN		//��ͬ��GPIO_ReadInputDataBit(IIC_GPIO,IIC_SDA_PIN)


#endif

//void I2C_GPIO_Config(void);
void My_I2C_Init(uint8_t ownaddr, uint32_t I2Cspeed);
u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);
u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr);
bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data);
u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite);

#endif


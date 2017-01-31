
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��usart.c
 * ��	��	��stm32����ͨ������
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.2.150418
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.24
 * ���༭	��2015.4.18
 * ��	ע	��
 **-------------------------------------------------------------------------------

 * ��	��		��Damm Stanger
 * ��	��		��dammstanger@qq.com
**********************************************************************************************/

#include 	"usart.h"
#include "debug.h"

#define USART1_DR_Address  ((u32)0x40013804)


///////////////////////////////////////UART1����///////////////////////////////////////////////////////////////////

/*
 * ��������USART1_NVIC_Configuration
 * ����  ��USART1 �ж����ȼ�����
 * ����  ����
 * ���  ����
 */
void USART1_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  													
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������DMA1CH5_NVIC_Configuration
 * ����  ��DMA1CH5�ж����ȼ�����
 * ����  ����
 * ���  ����	
 */
void DMA1CH5_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  			//��ռ���ȼ�0λ											
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_Config(int BR)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);

	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.9) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	USART_Cmd(USART1, ENABLE);

	USART1->SR;			//����һ���ȡSR�Ĵ���  �����TC��־λ	ʹ���͵����ַ�����ʧ
	
	USART1_NVIC_Configuration();
}


//void USART1_Config(int BR)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	/* config USART1 clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1
//						  |RCC_APB2Periph_GPIOB
//						  |RCC_APB2Periph_AFIO, ENABLE);

//	/* USART1 remapIO config */
//	GPIO_PinRemapConfig(GPIO_Remap_USART1 , ENABLE);

//	/* USART1 GPIO config */
//	/* Configure USART1 Tx (PB.6) as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);    
//	/* Configure USART1 Rx (PB.7) as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	USART1_DMAConfig();

//	/* USART1 mode config */
//	USART_InitStructure.USART_BaudRate = BR;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART1, &USART_InitStructure); 
////	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);			//ʹ��DMA��Ͳ�Ҫʹ�ܽ����ж�
//		
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//	USART_Cmd(USART1, ENABLE);
//	
//}

DMA_InitTypeDef DMA_InitStructure;

void USART1_DMAConfig(u8 *buf)
{
//	DMA_InitTypeDef DMA_InitStructure;

	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* DMA channel1ͨ��5 configuration */
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf; 							//DMAѡ��洢��Ԫ�Ļ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 12;						 				//12�ֽ�
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									//��ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);									//adc��ͨ��5
	
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);									//ʹ��DMA1CH5����ж�
	DMA_ClearITPendingBit(DMA1_IT_TC5);

	/* Enable DMA channel5 */
	DMA_Cmd(DMA1_Channel5, ENABLE);
}


void USART1_SendData(unsigned char dat)
{
	
	USART_SendData(USART1,dat);
	while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );
}

void USART1_SendStr(unsigned char *str,unsigned char len)
{
	u8 i;
	for(i=0;i<len;i++)
	{
		USART1_SendData(*str);
		str++;
	}
}


void USART1_Senddugstr(unsigned char *str)
{
	while(*str!='\0')
	{
		USART1_SendData(*str);
		str++;
	}
}

void USART1_IRQHandler()
{	
	u8 sbuf;
	sbuf = USART_ReceiveData(USART1);
	//�û����ݴ���
	Dug_Pkg_PIDPar_Handle(sbuf);
	USART_ClearFlag(USART1,USART_FLAG_RXNE);
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

void delay(u8 n)
{
	for(;n>0;n--);
}

void DMA1_Channel5_IRQHandler(void)
{
//	if(DMA_GetITStatus(DMA1_IT_TC5)==SET)
//	{
//		DMA_ClearITPendingBit(DMA1_IT_TC5);
//		if(PakRev_DMAHandle()==FALSE)
//		{	
//			USART_Cmd(USART1, DISABLE);
//			USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);

//			DMA_Cmd(DMA1_Channel5, DISABLE);		  //����ͣDMA,�������üĴ���
//			DMA1_Channel5->CNDTR = 12;
//			USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//			DMA_Cmd(DMA1_Channel5, ENABLE);
//						
//			USART_Cmd(USART1, ENABLE);
//		}
//		else
//		{
//			DMA_Cmd(DMA1_Channel5, DISABLE);		  //����ͣDMA,�������üĴ���
//			DMA1_Channel5->CNDTR = 12;
//			DMA_Cmd(DMA1_Channel5, ENABLE);	
//		}
//	}
}

//------------------------------------
//			USART_Cmd(USART1, DISABLE);
//			DMA_Cmd(DMA1_Channel5, DISABLE);		  
//			DMA1_Channel5->CNDTR = 12;
//			USART_Cmd(USART1, ENABLE);				
//			DMA_Cmd(DMA1_Channel5, ENABLE);		  	//����һ�´��ڿ��Ա��⴮�ڻ�������������DMA���������DMA����ɼ�����CNTR��һ��
													//��ȫ��ִ��ʱ������Ч��Ӧ��Ҫ����ʱ���ж��в������ӡ�

//------------------------------------


/********************************************************************************
 * ��������USART1_EnableRevDMA()
 * ����  ������DMA������DMA��������ֵ
 * ����  ��-		    	
 * ����  ��-
 * ����  ���ⲿ����
 ********************************************************************************/
void USART1_EnableRevDMA()
{
	DMA_Cmd(DMA1_Channel5, DISABLE);		  //����ͣDMA
	DMA1_Channel5->CNDTR = 12;
	DMA_Cmd(DMA1_Channel5, ENABLE);	
}


#ifndef __cplusplus

/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
int fputc(int ch, FILE *f)
{
/* ��Printf���ݷ������� */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

#endif



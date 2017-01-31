
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：usart.c
 * 描	述	：stm32串口通信驱动
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.2.150418
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2015.4.18
 * 备	注	：
 **-------------------------------------------------------------------------------

 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/

#include 	"usart.h"
#include "debug.h"

#define USART1_DR_Address  ((u32)0x40013804)


///////////////////////////////////////UART1部分///////////////////////////////////////////////////////////////////

/*
 * 函数名：USART1_NVIC_Configuration
 * 描述  ：USART1 中断优先级配置
 * 输入  ：无
 * 输出  ：无
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
 * 函数名：DMA1CH5_NVIC_Configuration
 * 描述  ：DMA1CH5中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */
void DMA1CH5_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  			//先占优先级0位											
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

	USART1->SR;			//加上一句读取SR寄存器  清除了TC标志位	使发送的首字符不丢失
	
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
////	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);			//使用DMA后就不要使能接收中断
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
	
	/* DMA channel1通道5 configuration */
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buf; 							//DMA选择存储单元的基址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 12;						 				//12字节
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									//非循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);									//adc在通道5
	
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);									//使能DMA1CH5完成中断
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
	//用户数据处理
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

//			DMA_Cmd(DMA1_Channel5, DISABLE);		  //先暂停DMA,才能设置寄存器
//			DMA1_Channel5->CNDTR = 12;
//			USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//			DMA_Cmd(DMA1_Channel5, ENABLE);
//						
//			USART_Cmd(USART1, ENABLE);
//		}
//		else
//		{
//			DMA_Cmd(DMA1_Channel5, DISABLE);		  //先暂停DMA,才能设置寄存器
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
//			DMA_Cmd(DMA1_Channel5, ENABLE);		  	//重启一下串口可以避免串口缓冲区的数据在DMA重启后进入DMA，造成计数器CNTR减一，
													//但全速执行时重启无效，应需要加延时，中断中不能拖延。

//------------------------------------


/********************************************************************************
 * 函数名：USART1_EnableRevDMA()
 * 描述  ：重启DMA，并给DMA计数器赋值
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：外部调用
 ********************************************************************************/
void USART1_EnableRevDMA()
{
	DMA_Cmd(DMA1_Channel5, DISABLE);		  //先暂停DMA
	DMA1_Channel5->CNDTR = 12;
	DMA_Cmd(DMA1_Channel5, ENABLE);	
}


#ifndef __cplusplus

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

#endif



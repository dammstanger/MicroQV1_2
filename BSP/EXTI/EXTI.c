/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：EXTI.c
 * 描	述	：外部中断初始化和中断服务
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150412
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.11
 * 最后编辑	：2015.4.12
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/



/****************************包含头文件*******************************************/

#include "nrf24l01.h"
#include "MPU9250.h"
#include "../uCOS-II/Source/ucos_ii.h"
#include "EXTI.h"
/****************************宏定义***********************************************/

/****************************变量声明*********************************************/
extern OS_EVENT * msg_nrf;				//NRF信号量事件块指针
extern OS_EVENT * g_Msgsem_rawdatrdy;					//原始数据采集好
/****************************变量定义*********************************************/

/****************************函数声明*********************************************/




/********************************************************************************
 * 函数名：EXTI_PA11_24L01_Init()
 * 描述  ：配置用于24L01的IRQ外部中断
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：外部
 ********************************************************************************/
void EXTI_PA11_24L01_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);

	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);										//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;				//使能IRQ所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;					//子优先级5
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 	

}


//外部中断0服务程序 
void EXTI15_10_IRQHandler(void)
{	
	OSSemPost(msg_nrf);
	EXTI_ClearITPendingBit(EXTI_Line11); //清除LINE11上的中断标志位  
}


//外部中断0服务程序 
void EXTI9_5_IRQHandler(void)
{	
//	OS_CPU_SR cpu_sr=0;
	OSIntEnter();
	MPU9250_Read();
	OSIntExit();						//退出临界区
	OSSemPost(g_Msgsem_rawdatrdy);
	EXTI_ClearITPendingBit(EXTI_Line5); //清除LINE5上的中断标志位  
}
		
/******************* (C) COPYRIGHT 2015 DammStanger *****END OF FILE************/

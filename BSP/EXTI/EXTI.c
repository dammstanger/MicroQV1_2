/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��EXTI.c
 * ��	��	���ⲿ�жϳ�ʼ�����жϷ���
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150412
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.11
 * ���༭	��2015.4.12
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/



/****************************����ͷ�ļ�*******************************************/

#include "nrf24l01.h"
#include "MPU9250.h"
#include "../uCOS-II/Source/ucos_ii.h"
#include "EXTI.h"
/****************************�궨��***********************************************/

/****************************��������*********************************************/
extern OS_EVENT * msg_nrf;				//NRF�ź����¼���ָ��
extern OS_EVENT * g_Msgsem_rawdatrdy;					//ԭʼ���ݲɼ���
/****************************��������*********************************************/

/****************************��������*********************************************/




/********************************************************************************
 * ��������EXTI_PA11_24L01_Init()
 * ����  ����������24L01��IRQ�ⲿ�ж�
 * ����  ��-		    	
 * ����  ��-
 * ����  ���ⲿ
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
	EXTI_Init(&EXTI_InitStructure);										//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;				//ʹ��IRQ���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;					//�����ȼ�5
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 	

}


//�ⲿ�ж�0������� 
void EXTI15_10_IRQHandler(void)
{	
	OSSemPost(msg_nrf);
	EXTI_ClearITPendingBit(EXTI_Line11); //���LINE11�ϵ��жϱ�־λ  
}


//�ⲿ�ж�0������� 
void EXTI9_5_IRQHandler(void)
{	
//	OS_CPU_SR cpu_sr=0;
	OSIntEnter();
	MPU9250_Read();
	OSIntExit();						//�˳��ٽ���
	OSSemPost(g_Msgsem_rawdatrdy);
	EXTI_ClearITPendingBit(EXTI_Line5); //���LINE5�ϵ��жϱ�־λ  
}
		
/******************* (C) COPYRIGHT 2015 DammStanger *****END OF FILE************/

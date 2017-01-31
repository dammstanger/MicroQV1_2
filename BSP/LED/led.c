/****************************************Copyright (c)****************************************************
**                                       �����Ƽ���ѧ
**                                     
**                                         ����Э��
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           delay.c
** Last modified Date: 
** Last Version: 
** Description:         DX���������LED����Դ�ļ�
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          ����
** Created date:        2013/11/09
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			dammstanger	
** Modified date:		2015.6.23
** Version:
** Description:			������LEDTrg()����
**
*********************************************************************************************************/


/*********************************************************************************************************
	����ͷ�ļ�
*********************************************************************************************************/
#include "led.h"


/*********************************************************************************************************
** Function name:       LED_Init
** Descriptions:        LED��ʼ��
** input parameters:    none
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;				//���������ʼ���ṹ��
	
		//����ʧ��JTAG����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);		//ʹ������ʱ��

	GPIO_InitStructure.GPIO_Pin = LED1_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED1_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED2_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED2_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED3_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED3_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED4_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED4_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED5_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED5_GPIO, &GPIO_InitStructure);



}


/*********************************************************************************************************
** Function name:       LED_Set
** Descriptions:        LED����
** input parameters:    __led:		LEDѡ��(1~4)
**						__state:	LED״̬(1:��,0:��)
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void LED_Set(u8 __led, u8 __state)
{
	switch (__led)
	{
		case 1:
			if (!__state)
			{
				GPIO_SetBits(LED1_GPIO,LED1_Pin);
			}
			else
			{
				GPIO_ResetBits(LED1_GPIO,LED1_Pin);
			}
			break;
			
		case 2:
			if (!__state)
			{
				GPIO_SetBits(LED2_GPIO,LED2_Pin);
			}
			else
			{
				GPIO_ResetBits(LED2_GPIO,LED2_Pin);
			}
			break;
			
		case 3:
			if (!__state)
			{
				GPIO_SetBits(LED3_GPIO,LED3_Pin);
			}
			else
			{
				GPIO_ResetBits(LED3_GPIO,LED3_Pin);
			}
			break;
			
		case 4:
			if (!__state)
			{
				GPIO_SetBits(LED4_GPIO,LED4_Pin);
			}
			else
			{
				GPIO_ResetBits(LED4_GPIO,LED4_Pin);
			}
			break;
		case 5:
			if (!__state)
			{
				GPIO_SetBits(LED5_GPIO,LED5_Pin);
			}
			else
			{
				GPIO_ResetBits(LED5_GPIO,LED5_Pin);
			}
			break;
	}
}


void LEDTrg(u8 led)
{
	switch(led)
	{
		case 1 : {			if(GPIO_ReadOutputDataBit(LED1_GPIO,LED1_Pin)) LED_Set(1,1);
							else  LED_Set(1,0);}break;
		case 2 : {			if(GPIO_ReadOutputDataBit(LED2_GPIO,LED2_Pin)) LED_Set(2,1);
							else  LED_Set(2,0);}break;
		case 3 : {			if(GPIO_ReadOutputDataBit(LED3_GPIO,LED3_Pin)) LED_Set(3,1);
							else  LED_Set(3,0);}break;
		case 4 : {			if(GPIO_ReadOutputDataBit(LED4_GPIO,LED4_Pin)) LED_Set(4,1);
							else  LED_Set(4,0);}break;		
		case 5 : {			if(GPIO_ReadOutputDataBit(LED5_GPIO,LED4_Pin)) LED_Set(5,1);
							else  LED_Set(5,0);}break;		
						
	}
}


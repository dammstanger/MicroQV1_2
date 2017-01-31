/****************************************Copyright (c)****************************************************
**                                       辽宁科技大学
**                                     
**                                         电子协会
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           delay.c
** Last modified Date: 
** Last Version: 
** Description:         DX四轴飞行器LED驱动源文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          王恺
** Created date:        2013/11/09
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			dammstanger	
** Modified date:		2015.6.23
** Version:
** Description:			增加了LEDTrg()函数
**
*********************************************************************************************************/


/*********************************************************************************************************
	包含头文件
*********************************************************************************************************/
#include "led.h"


/*********************************************************************************************************
** Function name:       LED_Init
** Descriptions:        LED初始化
** input parameters:    none
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;				//定义外设初始化结构体
	
		//配置失能JTAG功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);		//使能外设时钟

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
** Descriptions:        LED设置
** input parameters:    __led:		LED选择(1~4)
**						__state:	LED状态(1:亮,0:灭)
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


/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：debug.h
 * 描	述	：通过串口上传或下载数据_发送命令端
 *                   
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150418
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2015.4.18
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#ifndef __DEBUG_H_
#define __DEBUG_H_

/****************************包含头文件*******************************************/
#include "stm32f10x.h"
#include "usart.h"
#include "string.h"

#define PID_CTLTYPE_RPSTB		1			//roll pitch stabilitze controller
#define PID_CTLTYPE_RPA			2			//roll pitch angle controller
#define PID_CTLTYPE_RPARATE		3			//roll pitch angle rate controller
#define PID_CTLTYPE_YAW			4
#define PID_CTLTYPE_ALT			5
#define PID_CTLTYPE_POS			6
#define PID_CTLTYPE_ALTRATE		7
#define PID_CTLTYPE_ALTACC		8

extern BOOL g_UART_PIDPkg_Rev_Fin;

#define USART1_DEBUG
#ifdef USART1_DEBUG

extern uint8_t dug_buf1[128];

#define Dug_printf(format, args...) do{printf(format,##args);}while(0);
#else
#define Dug_printf(...) do{}while(0)
#endif

void Dug_Pkg_2401_Hunter(u8 *buf,int dat1,int dat2,int dat3,int dat4);//发送匹配串口猎人的数据
void SendDebugDat_SSCOM32(int dat1,int dat2,int dat3,int dat4);	//发送匹配SSCOM32的数据
void SendDebugDat_Hunter(int dat1,int dat2,int dat3,int dat4);//发送匹配串口猎人的数据
void Dug_Pkg_PIDPar_Handle(u8 dat);
	
	
	
#endif


/******************* (C) COPYRIGHT 2016 DammStanger *****END OF FILE************/


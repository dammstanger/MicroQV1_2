/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��debug.h
 * ��	��	��ͨ�������ϴ�����������_���������
 *                   
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150418
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.24
 * ���༭	��2015.4.18
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __DEBUG_H_
#define __DEBUG_H_

/****************************����ͷ�ļ�*******************************************/
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

void Dug_Pkg_2401_Hunter(u8 *buf,int dat1,int dat2,int dat3,int dat4);//����ƥ�䴮�����˵�����
void SendDebugDat_SSCOM32(int dat1,int dat2,int dat3,int dat4);	//����ƥ��SSCOM32������
void SendDebugDat_Hunter(int dat1,int dat2,int dat3,int dat4);//����ƥ�䴮�����˵�����
void Dug_Pkg_PIDPar_Handle(u8 dat);
	
	
	
#endif


/******************* (C) COPYRIGHT 2016 DammStanger *****END OF FILE************/


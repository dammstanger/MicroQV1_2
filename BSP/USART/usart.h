
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��usart.h
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

#ifndef USART_H
#define USART_H

#include "Project_cfg.h"
#include "stm32f10x.h"
#include <stdio.h>


extern u8 ReceiveData[];
extern u8 DebugData[20];


void USART1_NVIC_Configuration(void);
void DMA1CH5_NVIC_Configuration(void);
void USART1_Config(int BR);
void USART1_DMAConfig(u8 *buf);
void USART1_EnableRevDMA(void);

void USART1_SendData(unsigned char dat);
void USART1_SendStr(unsigned char *str,unsigned char len);
void USART1_Senddugstr(unsigned char *str);

void SendDebugDat_SSCOM32(int dat1,int dat2,int dat3,int dat4);	//����ƥ��SSCOM32������
void SendDebugDat_Hunter(int dat1,int dat2,int dat3,int dat4);//����ƥ�䴮�����˵�����

void USARTPIDDebug(u8 *dat);

//int fputc(int ch, FILE *f);

#endif

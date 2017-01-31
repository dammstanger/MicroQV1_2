/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：debug.c
 * 描	述	：通过串口调试,接收的PID参数包高位在前,顺序P-D-I
 *                   
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150418
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2015.4.18
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/


/****************************包含头文件*******************************************/
#include "debug.h"
#include "nrf24l01.h"
#include "ucos_ii.h"
#include "fly.h"
#include "DataProcess.h"
#include "Control.h"
/****************************宏定义***********************************************/

#define DEBUGDATASIZE 	10
#define REVDATASIZE		10
/****************************变量声明*********************************************/

/****************************变量定义*********************************************/


u8 DebugData[20] = {0x55,0,0,0,0,0,0,0,0,0xaa,0,0,0,0,0,0,0,0,0,0};
u8 ReceiveData[REVDATASIZE] = {0,0,0,0,0,0,0,0,0,0};
BOOL g_UART_PIDPkg_Rev_Fin = FALSE;

#ifdef USART1_DEBUG
uint8_t dug_buf1[128]={0};
#endif
/****************************函数声明*********************************************/




/********************************************************************************
 * 函数名：
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：- 
 * 调用  ：外部调用
 ********************************************************************************/


void Dug_Pkg_2401_Hunter(u8 *buf,int dat1,int dat2,int dat3,int dat4)//发送匹配串口猎人的数据
{
	s16 i;
	
	buf[0] = 0x55;
	i=(s16)dat1;
	buf[1] = (u8)(i>>8);
	buf[2] = (u8)(i&0xff);
	
	i=(s16)dat2;
	buf[3] = (u8)(i>>8);
	buf[4] = (u8)(i&0xff);
	
	i=(s16)dat3;
	buf[5] = (u8)(i>>8);
	buf[6] = (u8)(i&0xff);
	
	i=(s16)dat4;
	buf[7] = (u8)(i>>8);
	buf[8] = (u8)(i&0xff);
	buf[9] = 0xaa;				
}



void SendDebugDat_SSCOM32(int dat1,int dat2,int dat3,int dat4)	//发送匹配SSCOM32的数据
{	int i=0;
	
	i=(int)(dat1);	
	if(i<0)
	{
		USART1_SendData('-');
		i=-i;				
	}
	else
	{
		USART1_SendData(' ');
	}
	USART1_SendData((i/10000)%10+'0');
	USART1_SendData((i/1000)%10+'0');
	USART1_SendData((i/100)%10+'0');
	USART1_SendData((i/10)%10+'0');
	USART1_SendData((i/1)%10+'0');
	USART1_SendData(' ');
	
	i=(int)(dat2);	
	if(i<0)
	{
		USART1_SendData('-');
		i=-i;				
	}
	else
	{
		USART1_SendData(' ');
	}
	USART1_SendData((i/10000)%10+'0');
	USART1_SendData((i/1000)%10+'0');
	USART1_SendData((i/100)%10+'0');
	USART1_SendData((i/10)%10+'0');
	USART1_SendData((i/1)%10+'0');
	USART1_SendData(' ');
	
	i=(int)dat3;
	if(i<0)
	{
		USART1_SendData('-');
		i=-i;				
	}
	else
	{
		USART1_SendData(' ');
	}
	USART1_SendData((i/10000)%10+'0');
	USART1_SendData((i/1000)%10+'0');
	USART1_SendData((i/100)%10+'0');
	USART1_SendData((i/10)%10+'0');
	USART1_SendData((i/1)%10+'0');
	USART1_SendData(' ');
	
	i=(int)dat4;
	if(i<0)
	{
		USART1_SendData('-');
		i=-i;				
	}
	else
	{
		USART1_SendData(' ');
	}
	USART1_SendData((i/10000)%10+'0');
	USART1_SendData((i/1000)%10+'0');
	USART1_SendData((i/100)%10+'0');
	USART1_SendData((i/10)%10+'0');
	USART1_SendData((i/1)%10+'0');
	USART1_SendData(0x0d);
	USART1_SendData(0x0a);
}

void SendDebugDat_Hunter(int dat1,int dat2,int dat3,int dat4)//发送匹配串口猎人的数据
{
	s16 i;
	i=(s16)dat1;
	DebugData[1] = (u8)(i>>8);
	DebugData[2] = (u8)(i&0xff);
	
	i=(s16)dat2;
	DebugData[3] = (u8)(i>>8);
	DebugData[4] = (u8)(i&0xff);
	
	i=(s16)dat3;
	DebugData[5] = (u8)(i>>8);
	DebugData[6] = (u8)(i&0xff);
	
	i=(s16)dat4;
	DebugData[7] = (u8)(i>>8);
	DebugData[8] = (u8)(i&0xff);
	
	for(i=0;i<DEBUGDATASIZE;i++)
	{
		USART1_SendData(DebugData[i]);
	}	
}


u8 PIDDebugData(__fly_reg debdat)
{	
	u8 retval = 0;
	if(FlightMode==UNARMED)
	{
		switch(debdat.PIDPar_CTLTYPE_REG)
		{
			case PID_CTLTYPE_RPA :{
				PID_ROL.P = debdat.PIDPar_P_REG/1000.0;				//P项乘以1000，其余放大一万倍
				PID_PIT.P = PID_ROL.P;
				PID_ROL.I = debdat.PIDPar_I_REG/10000.0;
				PID_PIT.I = PID_ROL.I;
				PID_ROL.D = debdat.PIDPar_D_REG/10000.0;
				PID_PIT.D = PID_ROL.D;
				PID_ROL.Imax = debdat.PIDPar_IMAX_REG/10.0;
				PID_PIT.Imax = PID_ROL.Imax;
				retval = 0x22;
			}break;
			case PID_CTLTYPE_RPARATE :{
				PID_ROL_PIT_RATE.P = debdat.PIDPar_P_REG/1000.0;				//P项乘以1000，其余放大一万倍
				PID_ROL_PIT_RATE.I = debdat.PIDPar_I_REG/10000.0;
				PID_ROL_PIT_RATE.D = debdat.PIDPar_D_REG/10000.0;
				PID_ROL_PIT_RATE.Imax = debdat.PIDPar_IMAX_REG/10.0;
				retval = 0x33;
			}break;
			case PID_CTLTYPE_YAW :{
				PID_YAW.P = debdat.PIDPar_P_REG/1000.0;
				PID_YAW.I = debdat.PIDPar_I_REG/10000.0;
				PID_YAW.D = debdat.PIDPar_D_REG/10000.0;
				PID_YAW.Imax = debdat.PIDPar_IMAX_REG/10.0;
				retval = 0x44;
			}break;
			case PID_CTLTYPE_ALT :{
				PID_ALT.P = debdat.PIDPar_P_REG/1000.0;
				PID_ALT.I = debdat.PIDPar_I_REG/10000.0;
				PID_ALT.D = debdat.PIDPar_D_REG/10000.0;
				PID_ALT.Imax = debdat.PIDPar_IMAX_REG/10.0;	
				retval = 0x55;				
			}break;
			case PID_CTLTYPE_ALTRATE :{
				P_RATE_ALT.P = debdat.PIDPar_P_REG/1000.0;
				P_RATEFILT_ALT.P = debdat.PIDPar_I_REG/1000.0;
				retval = 0x77;
			}break;
			case PID_CTLTYPE_ALTACC :{
				PID_ACC_ALT.P = debdat.PIDPar_P_REG/1000.0;
				PID_ACC_ALT.I = debdat.PIDPar_I_REG/10000.0;
				PID_ACC_ALT.D = debdat.PIDPar_D_REG/10000.0;
				PID_ACC_ALT.Imax = debdat.PIDPar_IMAX_REG/10.0;
				retval = 0x88;
			}break;
			default : break;	
		}
		g_UART_PIDPkg_Rev_Fin = TRUE;
	}
	return retval;
}



//高位在前,顺序P-I-D
void Dug_PIDPkg_to_Flyreg(u8 *pkg,__fly_reg *flyreg)
{
	flyreg->PIDPar_CTLTYPE_REG = pkg[1];
	flyreg->PIDPar_P_REG = ((u16)pkg[2]<<8)+pkg[3];
	flyreg->PIDPar_I_REG = ((u16)pkg[4]<<8)+pkg[5];
	flyreg->PIDPar_D_REG = ((u16)pkg[6]<<8)+pkg[7];
	flyreg->PIDPar_IMAX_REG = ((u16)pkg[8]<<8)+pkg[9];
}

void Dug_Pkg_PIDPar_Handle(u8 dat)
{	
	static u8 check = 0,i= 0;
	
	if(check==2)
	{
		ReceiveData[i] = dat;
		i++;
		if(i==REVDATASIZE)
		{
			i = 0;
			check = 0;
			Dug_PIDPkg_to_Flyreg(ReceiveData,&FlyReg);
			PIDDebugData(FlyReg);
		}
	}
	if((dat==0x55)&&(check==0))	check = 1;
	if((dat==0xaa)&&(check==1)) {check = 2;i = 0;}
	
}


/******************* (C) COPYRIGHT 2016 DammStanger *****END OF FILE************/


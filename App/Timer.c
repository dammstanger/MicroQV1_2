/******************** (C) COPYRIGHT 2016 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Timer.c
 * 描	述	：STC 60S2 延时，定时，等时间函数	11.0592MHz
 *            
 * 实验平台	：自控水泵V2.0	
 * 硬件连接	：
 * 版 	本	：V0.1.160217
 * 从属关系	：PoolAuto
 * 库版本	：无
 * 创建时间	：2016.2.17
 * 最后编辑	：2016.2.17
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

/****************************包含头文件*******************************************/
#include "Timer.h"
/****************************宏定义***********************************************/

/****************************变量定义***********************************************/
u8 Timer_reg=0;
u32 Timer1_cnt = 0;			//最小单位1ms,最大定时2^32秒
u32 Timer2_cnt = 0;
u32 Timer3_cnt = 0;
u32 Timer4_cnt = 0;

/********************************************************************************
 * 函数名：ReadSoftTimer
 * 描述  ：读取软件定时器寄存器
 * 输入  ：-		    	
 * 返回  ：- 
 * 调用  ：外部调用
 ********************************************************************************/
u8 ReadSoftTimer(u8 timerx)
{
	switch(timerx)
	{
		case TIMER_1	:{return (Timer_reg&TIMEUP_1);}
		case TIMER_2	:{return (Timer_reg&TIMEUP_2);}
		case TIMER_3	:{return (Timer_reg&TIMEUP_3);}	
		case TIMER_4	:{return (Timer_reg&TIMEUP_4);}	
		default	:break;					
	}
	return 0;
}

/********************************************************************************
 * 函数名：
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：- 
 * 调用  ：外部调用
 ********************************************************************************/
void SetSoftTimer(u8 timerx,long val)
{
	switch(timerx)
	{
		case TIMER_1	:{
			Timer_reg &=~0x01;Timer1_cnt = val;		//定时标记清零，计时器赋值
			}break;
		case TIMER_2	:{
			Timer_reg &=~0x02;Timer2_cnt = val;			
			}break;
		case TIMER_3	:{
			Timer_reg &=~0x04;Timer3_cnt = val;			
			}break;
		case TIMER_4	:{
			Timer_reg &=~0x08;Timer4_cnt = val;			
			}break;
		default	:break;			
	}
}

void SoftTimer()
{
	if(Timer1_cnt)	Timer1_cnt--;
	else 		Timer_reg |= 0x01;
	if(Timer2_cnt)	Timer2_cnt--;
	else 		Timer_reg |= 0x02;	
	if(Timer3_cnt)	Timer3_cnt--;
	else 		Timer_reg |= 0x04;
	if(Timer4_cnt)	Timer4_cnt--;
	else 		Timer_reg |= 0x08;
}
	
/******************* (C) COPYRIGHT 2016 DammStanger *****END OF FILE************/


/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：DataProcess.c
 * 描	述	：生成飞行模式，并依据飞行模式对各类数据进行处理，产生不同的控制目标
 *                     
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150616
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.1
 * 最后编辑	：2015.6.16
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

/****************************包含头文件*******************************************/
#include "math.h"
#include "DataProcess.h"
#include "AHRS.h"
#include "Attitude.h"
#include "Control.h"
/****************************变量定义*********************************************/
u8	FlightMode = UNARMED;
u8	FlyTask	   = TASK_TAKEOFF;
u8	ThrottleMode  = THROTTLE_MANUAL;
u8 VehicleBalComp = 0;
float temp_roll_view;


//-----------------高度控制相关参数------------------
u16	Control_proc_thro = 0;				//油门控制的中间量
u16 Control_auto_thro= 0;				//记录起飞的油门量
int32_t Control_targer_altacc = 0;			//高度ACC控制器目标加速度
BOOL g_altacc_controller_active = FALSE;

BOOL g_takeoff 			= FALSE;
BOOL g_takeoff_finish 	= FALSE;
BOOL g_land	  			= FALSE;
BOOL g_land_finish 		= FALSE;
u8	 g_land_detector	= 0;
u8   g_takeoff_2sdelay  = 0;				//用于电机自启动的延时

/****************************函数声明*********************************************/
u16 Thro_SlowStart(void);
void Fly_Task_Update(u8 flytask);
BOOL Update_Takeoff_Detector(void);
void Reset_Land_Detector(void);
void Reset_Takeoff_Detector(void);
u16 Thro_SlowDown(void);


/********************************************************************************
 * 函数名：UpdateFlightMode()
 * 描述  ：更新飞行模式
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void UpdateFlightMode(RC_DATA rcdata,__fly_reg flypar)
{
	if(rcdata.CH5<1500||(rcdata.THROTTLE>1400&&FlightMode==UNARMED))//||!(FlyReg.STATE_SYS_REG & STATE_SYS_RC_CONNECT))	//开关上锁，开关打开后如果油门大于危险值则不解锁
	{
		FlightMode = UNARMED;
		return ;
	}
	else
	{
		if(rcdata.THROTTLE<1100)										//解锁后油门小于等于危险值则解锁
		{
			FlightMode = ARMED;
			return ;	
		}
		else if((FlightMode==ARMED)&&(rcdata.THROTTLE>=1050))
		{
			FlightMode = ATTITUDE;
			return ;			
		}	
		else if(0)
		{
			FlightMode = ATLHOLD;
			return ;
		}
	}
}
float v_yaw_err;
/********************************************************************************
 * 函数名：FlyModeProcess()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void FlyModeProcess()
{
	UpdateFlightMode(Rc_Data,FlyReg);
	
	switch(FlightMode)
	{	
		case UNARMED :
			FlyReg.CONFIG_REG &= ~MOTOR_EN;
			break;
		case ARMED :
			FlyReg.CONFIG_REG &= ~MOTOR_EN;
			break;
		case ATTITUDE :
			{
				FlyReg.CONFIG_REG |= MOTOR_EN;
				ThrottleMode = THROTTLE_MANUAL;
				//ThrottleModeProcess();
				g_altacc_controller_active = FALSE;
			}break;
		case ATLHOLD :
			break;
	}
	
	Control_proc_att.yaw = Comput_Ctr_Yaw(Control_proc_att.yaw,Rc_Data);	
	v_yaw_err = Control_proc_att.yaw;
	Control_proc_att.roll 	= Control_proc_att.roll - RC_att.roll ;				//Control_proc_att在赋予姿态角度之后与期望的角度作差，
	Control_proc_att.pitch 	= Control_proc_att.pitch - RC_att.pitch;				
	Control_ulti_att = Control_proc_att;
}



/********************************************************************************
 * 函数名：ThrottleModeProcess()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void ThrottleModeProcess()
{
	s16 Control_tmp_thro =0;
	int32_t target_tmp_acc = 0;
	switch(ThrottleMode)
	{
		case THROTTLE_MANUAL:{
			if(FlightMode==ARMED||FlightMode==UNARMED)
			{
				Control_tmp_thro = 0;
			}
			else
			{
				Control_tmp_thro = Rc_Data.THROTTLE-690;
				if(Control_tmp_thro<0) Control_tmp_thro = 0;
				Control_tmp_thro = Control_tmp_thro*THRO_K;
			}
		}break;
	}
	Control_proc_thro = Control_tmp_thro;
	Control_targer_altacc = target_tmp_acc;
	if(Control_proc_thro>1600)	Control_proc_thro = 1600;
}

/********************************************************************************
 * 函数名：AttDataProcess()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void AttDataProcess()
{
	AHRS_Attitude();
}


/********************************************************************************
 * 函数名：Reset_Takeoff_Detector()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void Reset_Takeoff_Detector()
{
	g_takeoff_finish = FALSE;
}


/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

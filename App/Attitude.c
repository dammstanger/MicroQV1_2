/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Attitude.c
 * 描	述	：姿态处理
 *                    
 * 实验平台	：ano
 * 硬件连接	：
 * 版 	本	：V1.0.150615
 * 从属关系	：小四轴V0.0.2
 * 库版本	：ST3.0.0
 * 创建时间	：2014.6.23
 * 最后编辑	：2015.6.23
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************/

/****************************包含头文件*******************************************/
#include "ucos_ii.h"
#include "Attitude.h"
#include "math.h"
#include "Control.h"


/****************************宏定义***********************************************/
#define ALT_CURVE_PAR				170
#define ALT_CLIMBRATE_MAX			1000			//单位mm/s 即1米/秒 
#define ALT_DESCENTRATE_MAX			500

#define RC_DAEDBAND					70				//RC的单侧死区宽度
#define RC_ANGLE_RATIO				17				//死区=70，RC控制角度：430/17=25度
/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
//-航向处理变量------------------------------
BOOL Saveheadholdflag = FALSE;
BOOL headfreeflag = FALSE;
float Yaw_turn	= 0;
float Headhold = 0;
float Headoriginal_rad=0;

ZHOUDATA_int Vehiclebias={0,0,0};		//-416			//机体平衡位置修正值

EULER_DATA_TYPE RC_att;
EULER_DATA_TYPE Optflow_att;
EULER_DATA_TYPE Control_proc_att;	//控制过程的姿态
EULER_DATA_TYPE Control_ulti_att;	//ultimatez最终的控制角度

float Control_proc_yaw_rate;		//航向目标角速度
s32 Hight_i=0;						//高度PID


/********************************************************************************
 * 函数名：RCdat_T_Angle()
 * 描述  ：遥控输出的角度
 * 输入  ：
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
s16 RCdat_T_Angle(u16 rc_dat,s16 att_bias_axis)
{
	s16 angle = 0;
	s16 dead_min=1500-RC_DAEDBAND,dead_max=1500+RC_DAEDBAND;
	if(rc_dat<dead_min)
		angle = (rc_dat - dead_min +att_bias_axis)/17; //横滚量差值 = 当前横量 -（遥控值-1500）*一个比例系数 (相当于目标角)15步进
	else if(rc_dat>dead_max)
		angle = (rc_dat - dead_max+att_bias_axis)/17; 
	else angle= att_bias_axis/17; 	

	return angle;
}


/********************************************************************************
 * 函数名：Comput_Ctr_Yaw()
 * 描述  ：计算各种模式下航向控制量
 * 输入  ：   
 * 返回  ：输出目标角度		 
 * 调用  ：-
 ********************************************************************************/
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand)
{	
	float yawerror = 0;

		if(FlightMode==UNARMED||FlightMode==ARMED)		//进入头部锁定
		{
			Saveheadholdflag = FALSE;	//退出头部锁定模式时，保存标志清零，以备下次使用。
		}
		else
		{
			if(Saveheadholdflag==0)			//进入飞行，初始化当前机头锁定角和输入角度历史值
			{
				Headhold = input_yaw;		//转动前保存当前值
				Saveheadholdflag = TRUE;	//标记以保存
			}		
			if(RCcomand.YAW<1450||RCcomand.YAW>1550)	
			{
				Yaw_turn = -((RCcomand.YAW-1500)/1500.0);
			}
			else Yaw_turn = 0;	
			
			Headhold += Yaw_turn;
			
			if(Headhold>180)	Headhold = -360.0+Headhold;
			else if(Headhold<-180) Headhold = 360.0+Headhold;
			//-----------------------------------------------
			if(0<Headhold&&Headhold<=180)
			{
				if(input_yaw>(Headhold-180)&&input_yaw<=180) yawerror = Headhold-input_yaw;
				else yawerror = Headhold-input_yaw-360;
			}
			else
			{
				if(input_yaw<(Headhold+180)&&input_yaw>=-180) yawerror = Headhold-input_yaw;
				else yawerror = Headhold-input_yaw+360;
			}
		}
	return 	yawerror;	

	
}


/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

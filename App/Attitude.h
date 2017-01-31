/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Attitude.h
 * 描	述	：姿态处理
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.15
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

/****************************包含头文件*******************************************/
#include "stm32f10x.h"
#include "project_cfg.h"
#include "./RC/rc.h"
#include "AHRS.h"
/****************************宏定义***********************************************/


/****************************结构体定义*******************************************/
typedef struct 
{
	int32_t x;
	int32_t y;
	int32_t z;
}AXIS_DATA;

typedef struct 
{
//	int32_t roll;
//	int32_t	pitch;
//	int32_t yaw;
	float	roll;
	float	pitch;
	float	yaw;
}EULER_DATA_TYPE;

/****************************变量声明*********************************************/
extern s32 Hight_i;
extern float Headhold;
extern float Headoriginal_rad;
extern EULER_DATA_TYPE RC_att;
extern EULER_DATA_TYPE Optflow_att;
extern EULER_DATA_TYPE Control_proc_att;
extern EULER_DATA_TYPE Control_ulti_att;	//ultimatez最终的控制角度
extern float Control_proc_yaw_rate;		//航向目标角速度
extern ZHOUDATA_int Vehiclebias;
/****************************函数声明*********************************************/
s16 RCdat_T_Angle(u16 rc_dat,s16 att_bias_axis);
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand);


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


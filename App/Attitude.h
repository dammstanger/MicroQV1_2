/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Attitude.h
 * ��	��	����̬����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.15
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

/****************************����ͷ�ļ�*******************************************/
#include "stm32f10x.h"
#include "project_cfg.h"
#include "./RC/rc.h"
#include "AHRS.h"
/****************************�궨��***********************************************/


/****************************�ṹ�嶨��*******************************************/
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

/****************************��������*********************************************/
extern s32 Hight_i;
extern float Headhold;
extern float Headoriginal_rad;
extern EULER_DATA_TYPE RC_att;
extern EULER_DATA_TYPE Optflow_att;
extern EULER_DATA_TYPE Control_proc_att;
extern EULER_DATA_TYPE Control_ulti_att;	//ultimatez���յĿ��ƽǶ�
extern float Control_proc_yaw_rate;		//����Ŀ����ٶ�
extern ZHOUDATA_int Vehiclebias;
/****************************��������*********************************************/
s16 RCdat_T_Angle(u16 rc_dat,s16 att_bias_axis);
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand);


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


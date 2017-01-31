/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��DataProcess.h
 * ��	��	�����ɷ���ģʽ�������ݷ���ģʽ�Ը������ݽ��д���������ͬ�Ŀ���Ŀ��
 *                     
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150616
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.1
 * ���༭	��2015.6.16
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __DATAPROCESS_H_
#define	__DATAPROCESS_H_
/****************************����ͷ�ļ�*******************************************/
#include "Project_cfg.h"
#include "stm32f10x.h"
#include "AHRS.h"
//#include "RCdata.h"
/****************************�궨��***********************************************/

//-------------����ģʽ-----------------
#define UNARMED			0
#define	ARMED			1
#define ATTITUDE		2
#define	HEADFREE		3
#define ATLHOLD			4
#define POSHOLD			5
#define AUTO			6

//-----��������һ��������ģʽ��ʹ��-----------

#define TASK_TAKEOFF						1
#define TASK_POSHOLD						2
#define TASK_LAND							3

//---------------����ģʽ----------------------
#define THROTTLE_MANUAL						0
#define THROTTLE_HOLD                       1   // 
#define THROTTLE_AUTO                       2   // auto pilot altitude controller with target altitude held in cmd
#define THROTTLE_LAND                       3   // landing throttle controller

#define THRO_K				1.2				//��������б��

#define LAND_DETECTOR_CNT			15					//1.5S ȷ�Ͻ������

/****************************�ṹ�嶨��*******************************************/

/****************************��������*********************************************/
extern ZHOUDATA_f MOVEVEL;					//������ƶ����ٶ�	
extern ZHOUDATA_f MOVEACC;

//extern float sinx,siny,sin2x,sin2y,abs_Roll_Raw,abs_Pitch_Raw;
extern float vx,vy,vz;
extern u8 VehicleBalComp;
extern u8	FlightMode;						//����ģʽ
extern u16	Control_proc_thro;				//������
extern int32_t Control_targer_altacc;		//�߶�ACC������Ŀ����ٶ�
extern BOOL g_altacc_controller_active;
extern BOOL g_land_finish;
extern float v_yaw_err;
/****************************��������*********************************************/
void AttDataProcess(void);
void FlyModeProcess(void);
void ThrottleModeProcess(void);
 void Update_Land_Detector(void);
#endif

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

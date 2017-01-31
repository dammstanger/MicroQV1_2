
/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��RCdata.h
 * ��	��	����ң�ص����ݽ��д���
 *                    
 * ʵ��ƽ̨	��FCUV1.0
 * Ӳ������	��
 * �� 	��	��V1.0
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.25
 * ���༭	��2014.12.25
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

#ifndef _RCDATA_H_
#define _RCDATA_H_
/****************************����ͷ�ļ�*******************************************/
#include "stm32f10x.h"
#include "Project_cfg.h"

/****************************�궨��***********************************************/


/****************************�ṹ�嶨��*******************************************/
typedef struct 
{
	u16 ROLL;
	u16 PITCH;
	u16 THROTTLE;
	u16 YAW;
	u16 CH5;
	u16 CH6;

}RC_DATA;


/****************************��������*********************************************/
extern RC_DATA Rc_Data_temp;
extern RC_DATA Rc_Data;
extern RC_DATA Rc_DataLast;
extern RC_DATA Rc_Orign;
/****************************��������*********************************************/


void RCDataProcess(void);

#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/




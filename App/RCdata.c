/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��RCdata.c
 * ��	��	����ң�ص����ݽ��д���
 *                    
 * ʵ��ƽ̨	��ANO
 * Ӳ������	��
 * �� 	��	��V1.0
 * ��汾	��ST3.0.0
 * ����ʱ��	��2015.6.23
 * ���༭	��2015.6.23
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "RCdata.h"
/****************************�궨��***********************************************/

/****************************��������*********************************************/

/****************************��������*********************************************/
RC_DATA Rc_Data={1500,1500,1100,1000,1000,1500};
				//roll pit throt yaw CH5 CH6
RC_DATA Rc_DataLast={1500,1500,1100,1500,1000,1500};

RC_DATA Rc_Orign={0,0,0,0,0,0};

/******************************************************************************
/ ��������:RC���ݴ���
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
RC_DATA Rc_Data_temp = {0};
void RCDataProcess()
{		
	if(TIM2_Cap1STA&0x80) Rc_Data_temp.ROLL =     TIM2_Cap1Val;	//TIM2��������Ϊ1us��1000�պ�1ms,ֱ�Ӹ���
	if(TIM2_Cap2STA&0x80) Rc_Data_temp.PITCH =    TIM2_Cap2Val;
	if(TIM2_Cap3STA&0x80) Rc_Data_temp.THROTTLE = TIM2_Cap3Val;
	if(TIM2_Cap4STA&0x80) Rc_Data_temp.YAW =      TIM2_Cap4Val;
	if(TIM3_Cap3STA&0x80) Rc_Data_temp.CH5 =      TIM3_Cap3Val;
	if(TIM3_Cap4STA&0x80) Rc_Data_temp.CH6 =      TIM3_Cap4Val;	
	
	Rc_Data = Rc_Data_temp;
	
	if(Rc_Data.ROLL==0||Rc_Data.ROLL>2000)	  Rc_Data.ROLL = Rc_DataLast.ROLL;
	if(Rc_Data.PITCH==0||Rc_Data.PITCH>2000) Rc_Data.PITCH = Rc_DataLast.PITCH;
	if(Rc_Data.THROTTLE==0||Rc_Data.THROTTLE>2000) Rc_Data.THROTTLE = Rc_DataLast.THROTTLE;
	if(Rc_Data.YAW==0||Rc_Data.YAW>2000)     Rc_Data.YAW = Rc_DataLast.YAW;
	if(Rc_Data.CH5==0||Rc_Data.CH5>2200)     Rc_Data.CH5 = Rc_DataLast.CH5;
	if(Rc_Data.CH6==0||Rc_Data.CH6>2000)     Rc_Data.CH6 = Rc_DataLast.CH6;
	
	Rc_DataLast.ROLL = Rc_Data.ROLL;
	Rc_DataLast.PITCH = Rc_Data.PITCH;
	Rc_DataLast.THROTTLE = Rc_Data.THROTTLE;
	Rc_DataLast.YAW = Rc_Data.YAW;
	Rc_DataLast.CH5 = Rc_Data.CH5;
	Rc_DataLast.CH6 = Rc_Data.CH6;	

}



/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


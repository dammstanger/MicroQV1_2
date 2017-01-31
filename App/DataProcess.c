/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��DataProcess.c
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

/****************************����ͷ�ļ�*******************************************/
#include "math.h"
#include "DataProcess.h"
#include "AHRS.h"
#include "Attitude.h"
#include "Control.h"
/****************************��������*********************************************/
u8	FlightMode = UNARMED;
u8	FlyTask	   = TASK_TAKEOFF;
u8	ThrottleMode  = THROTTLE_MANUAL;
u8 VehicleBalComp = 0;
float temp_roll_view;


//-----------------�߶ȿ�����ز���------------------
u16	Control_proc_thro = 0;				//���ſ��Ƶ��м���
u16 Control_auto_thro= 0;				//��¼��ɵ�������
int32_t Control_targer_altacc = 0;			//�߶�ACC������Ŀ����ٶ�
BOOL g_altacc_controller_active = FALSE;

BOOL g_takeoff 			= FALSE;
BOOL g_takeoff_finish 	= FALSE;
BOOL g_land	  			= FALSE;
BOOL g_land_finish 		= FALSE;
u8	 g_land_detector	= 0;
u8   g_takeoff_2sdelay  = 0;				//���ڵ������������ʱ

/****************************��������*********************************************/
u16 Thro_SlowStart(void);
void Fly_Task_Update(u8 flytask);
BOOL Update_Takeoff_Detector(void);
void Reset_Land_Detector(void);
void Reset_Takeoff_Detector(void);
u16 Thro_SlowDown(void);


/********************************************************************************
 * ��������UpdateFlightMode()
 * ����  �����·���ģʽ
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void UpdateFlightMode(RC_DATA rcdata,__fly_reg flypar)
{
	if(rcdata.CH5<1500||(rcdata.THROTTLE>1400&&FlightMode==UNARMED))//||!(FlyReg.STATE_SYS_REG & STATE_SYS_RC_CONNECT))	//�������������ش򿪺�������Ŵ���Σ��ֵ�򲻽���
	{
		FlightMode = UNARMED;
		return ;
	}
	else
	{
		if(rcdata.THROTTLE<1100)										//����������С�ڵ���Σ��ֵ�����
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
 * ��������FlyModeProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
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
	Control_proc_att.roll 	= Control_proc_att.roll - RC_att.roll ;				//Control_proc_att�ڸ�����̬�Ƕ�֮���������ĽǶ����
	Control_proc_att.pitch 	= Control_proc_att.pitch - RC_att.pitch;				
	Control_ulti_att = Control_proc_att;
}



/********************************************************************************
 * ��������ThrottleModeProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
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
 * ��������AttDataProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void AttDataProcess()
{
	AHRS_Attitude();
}


/********************************************************************************
 * ��������Reset_Takeoff_Detector()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Reset_Takeoff_Detector()
{
	g_takeoff_finish = FALSE;
}


/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

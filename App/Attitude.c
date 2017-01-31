/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Attitude.c
 * ��	��	����̬����
 *                    
 * ʵ��ƽ̨	��ano
 * Ӳ������	��
 * �� 	��	��V1.0.150615
 * ������ϵ	��С����V0.0.2
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.6.23
 * ���༭	��2015.6.23
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "ucos_ii.h"
#include "Attitude.h"
#include "math.h"
#include "Control.h"


/****************************�궨��***********************************************/
#define ALT_CURVE_PAR				170
#define ALT_CLIMBRATE_MAX			1000			//��λmm/s ��1��/�� 
#define ALT_DESCENTRATE_MAX			500

#define RC_DAEDBAND					70				//RC�ĵ����������
#define RC_ANGLE_RATIO				17				//����=70��RC���ƽǶȣ�430/17=25��
/****************************��������*********************************************/

/****************************��������*********************************************/
//-���������------------------------------
BOOL Saveheadholdflag = FALSE;
BOOL headfreeflag = FALSE;
float Yaw_turn	= 0;
float Headhold = 0;
float Headoriginal_rad=0;

ZHOUDATA_int Vehiclebias={0,0,0};		//-416			//����ƽ��λ������ֵ

EULER_DATA_TYPE RC_att;
EULER_DATA_TYPE Optflow_att;
EULER_DATA_TYPE Control_proc_att;	//���ƹ��̵���̬
EULER_DATA_TYPE Control_ulti_att;	//ultimatez���յĿ��ƽǶ�

float Control_proc_yaw_rate;		//����Ŀ����ٶ�
s32 Hight_i=0;						//�߶�PID


/********************************************************************************
 * ��������RCdat_T_Angle()
 * ����  ��ң������ĽǶ�
 * ����  ��
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
s16 RCdat_T_Angle(u16 rc_dat,s16 att_bias_axis)
{
	s16 angle = 0;
	s16 dead_min=1500-RC_DAEDBAND,dead_max=1500+RC_DAEDBAND;
	if(rc_dat<dead_min)
		angle = (rc_dat - dead_min +att_bias_axis)/17; //�������ֵ = ��ǰ���� -��ң��ֵ-1500��*һ������ϵ�� (�൱��Ŀ���)15����
	else if(rc_dat>dead_max)
		angle = (rc_dat - dead_max+att_bias_axis)/17; 
	else angle= att_bias_axis/17; 	

	return angle;
}


/********************************************************************************
 * ��������Comput_Ctr_Yaw()
 * ����  ���������ģʽ�º��������
 * ����  ��   
 * ����  �����Ŀ��Ƕ�		 
 * ����  ��-
 ********************************************************************************/
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand)
{	
	float yawerror = 0;

		if(FlightMode==UNARMED||FlightMode==ARMED)		//����ͷ������
		{
			Saveheadholdflag = FALSE;	//�˳�ͷ������ģʽʱ�������־���㣬�Ա��´�ʹ�á�
		}
		else
		{
			if(Saveheadholdflag==0)			//������У���ʼ����ǰ��ͷ�����Ǻ�����Ƕ���ʷֵ
			{
				Headhold = input_yaw;		//ת��ǰ���浱ǰֵ
				Saveheadholdflag = TRUE;	//����Ա���
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

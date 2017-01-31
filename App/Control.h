/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Controll.h
 * ��	��	����̬������
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150613
 * ������ϵ	��FCU_UCOSII150603_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.13
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/
/****************************����ͷ�ļ�*******************************************/
#ifndef __CONTROL_H_
#define	__CONTROL_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
#include "Attitude.h"
#include "fly.h"

#define MOTO_PWM_MAX 999		//������������ֵ
#define MOTO_PWM_MIN 0		//�����������Сֵ
//#define CALIBRATE_PWMOUT		

typedef struct 
{
	float P;
	float I;
	float D;
	float Pout;
	float Iout;
	float Dout;
	float Imax;
	float ALLout;
}PID_PAR;

typedef struct 
{
	float P;
	float I;
	float Pout;
	float Iout;
	float Imax;
	float ALLout;
}PI_PAR;

typedef struct 
{
	float P;
	float D;
	float Pout;
	float Dout;
	float ALLout;
}PD_PAR;

typedef struct 
{
	float P;
	float Pout;
}P_PAR;

extern PID_PAR PID_ALT,PID_ACC_ALT;
extern P_PAR P_SONAR_ALT;
extern P_PAR P_ROOT_ALT;
extern P_PAR P_RATE_ALT;
extern P_PAR P_RATEFILT_ALT;

extern s16 roll_out_int;
extern s16 v_tar_rate,v_rate_error,v_dout,v_pout;

extern PID_PAR PID_ROL,PID_PIT,PID_YAW,PID_ROL_PIT_RATE;
extern float Temp_thro;
extern s32 Hight_i;

void PID_Para_Init(void);
void CONTROL(EULER_DATA_TYPE ctr_angle,int32_t alt_target_acc,__fly_reg *flypar,BOOL enable);
float AltOld_Controller(void);

#endif

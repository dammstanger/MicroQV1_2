/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Controll.c
 * 描	述	：姿态控制器
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150613
 * 从属关系	：FCU_UCOSII150603_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.13
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
/****************************包含头文件*******************************************/
#include "Control.h"
#include "DataProcess.h"
#include ".\MOTOR\motor.h"
//----------------安全设置------------------------
#define LIMITANGLGE_MAX		 800		//单位0.1度
#define LIMITANGLGE_MIN		-800

#define CALPWM_T_MOTPWM_RATIO	1.0f
#define LIMITPWM_ROLL		450
#define LIMITPWM_PITCH		450
#define LIMITPWM_YAW		350
#define VALID_THRO_LEVEL	1080		//油门有效的起始值

#define Filter_D_ATT		7.9577e-3	//1/(2*pi*fcut)	fcut截止频率=20Hz

u8 Unlock = 0;							//解锁标志



float Target_rate_roll = 0;				//翻滚轴目标速率
float Target_rate_pitch = 0;			//俯仰轴目标速率

//-------------------------------------------------------------------------------
P_PAR 	P_SONAR_ALT;
P_PAR 	P_ROOT_ALT;
P_PAR 	P_RATE_ALT;
P_PAR 	P_RATEFILT_ALT;
PID_PAR PID_ALT,PID_ACC_ALT;
//-------att controller---------
PID_PAR PID_ROL,PID_PIT,PID_YAW;
PID_PAR PID_ROL_PIT_RATE;
PID_PAR PID_POS,PID_POS_OF;
float rol_i=0,pit_i=0,yaw_i=0;
//-----函数声明--------------------------------------------------------------------------
BOOL SafeDeal(s16 rol, s16 pit);
//-----高度处理变量--------------------------------------------------------------------------

//-------------------------------------------
//	PID_ROL.P = 14;
//	PID_ROL.D = 0;
//	PID_ROL.I = 0;
//	PID_ROL.Imax = 0;
//	
//	PID_PIT.P = 14;
//	PID_PIT.D = 0;
//	PID_PIT.I = 0;
//	PID_PIT.Imax = 0;
//	
//	PID_YAW.P = 1.8;			//2.0
//	PID_YAW.D = 2.3;			//2.5
////	PID_YAW.I = 0.00;
////	PID_YAW.Imax = 0;

//	PID_ROL_PIT_RATE.P = 1.5;		//2.0;
//	PID_ROL_PIT_RATE.D =0.35;		//0.35;
//	PID_ROL_PIT_RATE.I = 0;
//	PID_ROL_PIT_RATE.Imax = 0;
//	

//	PID_ROL_PIT_RATE.P = 4.0f;
//	PID_ROL_PIT_RATE.D =0.15f;
//	PID_ROL_PIT_RATE.I = 0;
//	PID_ROL_PIT_RATE.Imax = 0;
//	PID_ROL.ALLout = 0;
//	PID_PIT.ALLout = 0;
//	PID_YAW.ALLout = 0;
void PID_Para_Init()
{
	PID_ROL.P = 10.0f;
	PID_ROL.D = 0;
	PID_ROL.I = 0;
	PID_ROL.Imax = 0;
	
	PID_PIT.P = 10.0f;
	PID_PIT.D = 0;
	PID_PIT.I = 0;
	PID_PIT.Imax = 0;
	
	PID_YAW.P = 13.0f;			//2.0
	PID_YAW.D = 2.5f;			//2.5
	PID_YAW.I = 0.00f;
	PID_YAW.Imax = 0;

	PID_ROL_PIT_RATE.P = 3.0f;
	PID_ROL_PIT_RATE.D =0.15f;
	PID_ROL_PIT_RATE.I = 0;
	PID_ROL_PIT_RATE.Imax = 0;
	
	PID_ROL.ALLout = 0;
	PID_PIT.ALLout = 0;
	PID_YAW.ALLout = 0;
}

//16位整型数限幅
s16 constrain_int16(int16_t dat, int16_t low, int16_t high) {
	return ((dat)<(low)?(low):((dat)>(high)?(high):(dat)));
}


//低通滤波器 a=tao/(tao+dt) 周期远大于tao的信号可以无损失通过
float LP_Filter(float y_last,float y,float p_a)			
{
	return y_last*p_a + y*(1.0f-p_a);
}

s16 v_tar_rate,v_rate_error,v_dout,v_pout;

s16 Roll_Rate_Controller(float target_rate,BOOL rst_i)
{
	float rate_error,rate_err_rate;
	static float rate_error_last = 0;
	static float rate_err_rate_last = 0;
	float pout=0,dout=0;
	//p
	rate_error = target_rate - GYRO_Deg.Y;
	pout = PID_ROL_PIT_RATE.P * rate_error;
	//d
	rate_err_rate = rate_error - rate_error_last;
	rate_err_rate = LP_Filter(rate_err_rate_last,rate_err_rate,0.83f);		//a=tao/(tao+dt)=0.83,tao时间常数：100ms dt周期：2ms
	dout = PID_ROL_PIT_RATE.D *rate_err_rate*500.0f;
	//i
	if(PID_ROL_PIT_RATE.I!=0&&!rst_i)					//i系数不为零且i项复位不使能，开始计算i
	{
		PID_ROL_PIT_RATE.Iout += PID_ROL_PIT_RATE.I * rate_error * 0.002f;				//dt = 2ms	
		if(PID_ROL_PIT_RATE.Iout>PID_ROL_PIT_RATE.Imax) PID_ROL_PIT_RATE.Iout=PID_ROL_PIT_RATE.Imax;						//积分限幅
		if(PID_ROL_PIT_RATE.Iout<-PID_ROL_PIT_RATE.Imax)PID_ROL_PIT_RATE.Iout=-PID_ROL_PIT_RATE.Imax;
	}
	else{
		PID_ROL_PIT_RATE.Iout = 0;
	}
	
	rate_error_last = rate_error;
	rate_err_rate_last = rate_err_rate;
	
	v_tar_rate = target_rate;
	v_rate_error = rate_error;
	v_dout = dout;
	v_pout = pout;
	return (s16)(pout+dout);
}

s16 Pitch_Rate_Controller(float target_rate,BOOL rst_i)
{
	float rate_error,rate_err_rate;
	static float rate_error_last = 0;
	static float rate_err_rate_last = 0;
	float pout=0,dout=0;
	//p
	rate_error = target_rate - GYRO_Deg.X;
	pout = PID_ROL_PIT_RATE.P * rate_error;
	//d
	rate_err_rate = rate_error - rate_error_last;
	rate_err_rate = LP_Filter(rate_err_rate_last,rate_err_rate,0.83f);		//a=tao/(tao+dt)=0.83,tao时间常数：100ms dt周期：2ms
	dout = PID_ROL_PIT_RATE.D *rate_err_rate*500.0f;
	//i
	if(PID_ROL_PIT_RATE.I!=0&&!rst_i)					//i系数不为零且i项复位不使能，开始计算i
	{
		PID_ROL_PIT_RATE.Iout += PID_ROL_PIT_RATE.I * rate_error * 0.002f;				//dt = 2ms	
		if(PID_ROL_PIT_RATE.Iout>PID_ROL_PIT_RATE.Imax) PID_ROL_PIT_RATE.Iout=PID_ROL_PIT_RATE.Imax;						//积分限幅
		if(PID_ROL_PIT_RATE.Iout<-PID_ROL_PIT_RATE.Imax)PID_ROL_PIT_RATE.Iout=-PID_ROL_PIT_RATE.Imax;
	}
	else{
		PID_ROL_PIT_RATE.Iout = 0;
	}
	
	
	rate_error_last = rate_error;
	rate_err_rate_last = rate_err_rate;

	return (s16)(pout+dout);
}

	s16 roll_out_int,pitch_out_int,yaw_out_int;
/********************************************************************************
 * 函数名：CONTROL
 * 描述  ：
 * 输入  ：-
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
void CONTROL(EULER_DATA_TYPE ctr_angle,int32_t alt_target_acc,__fly_reg *flypar,BOOL enable)
{	
	static u8	altexecnt = 0;						//高度acc控制器执行计数器
	static u16  altacc_thro_save = 0;				//用于保持每一次高度ACC控制器的输出
	static float roll_last = 0;
	static float pitch_last = 0;
	static s16 throttle=0;
	BOOL failsafe = FALSE;
	
	failsafe = SafeDeal(flypar->ATTI_ROLL_REG,flypar->ATTI_PITCH_REG);
	
	
	//----------------------高度ACC控制器------------------------------
	if(altexecnt++==10)					
	{
		altexecnt = 0;
//		if(g_altacc_controller_active)
//		altacc_thro_save = AltAcc_Controller(alt_target_acc);		
	}
	
	if(g_altacc_controller_active)
		throttle += altacc_thro_save;
	else
		throttle = flypar->FLY_THRO_REG - VALID_THRO_LEVEL;
	if(throttle>1000) throttle = 1000;
	else if(throttle<0) throttle = 0;
	//
	throttle = (s16)(throttle*CALPWM_T_MOTPWM_RATIO);		//油门缩放到电机PWM范围
	//-------------------------------------------------------------------------------------------
	PID_ROL.Pout = PID_ROL.P * ctr_angle.roll;
	PID_ROL.Dout = PID_ROL.D * (ctr_angle.roll - roll_last) * 500;		//1/dt
	roll_last	 = ctr_angle.roll;
	if(PID_ROL.I!=0)
	{
		PID_ROL.Iout += PID_ROL.I * ctr_angle.roll * 0.002f;			//dt = 2ms	
		if(PID_ROL.Iout>PID_ROL.Imax) PID_ROL.Iout=PID_ROL.Imax;						//积分限幅
		if(PID_ROL.Iout<-PID_ROL.Imax)PID_ROL.Iout=-PID_ROL.Imax;
	}
	//-------------------------------------------------------------------------------------------
	PID_PIT.Pout = PID_PIT.P * ctr_angle.pitch;
	PID_PIT.Dout = PID_PIT.D * (ctr_angle.pitch - pitch_last) * 500;	//1/dt
	pitch_last	 = ctr_angle.pitch;
	if(PID_PIT.I!=0)
	{
		PID_PIT.Iout += PID_PIT.I * ctr_angle.pitch * 0.002f;			//dt = 2ms	
		if(PID_PIT.Iout>PID_PIT.Imax) PID_PIT.Iout=PID_PIT.Imax;						//积分限幅
		if(PID_PIT.Iout<-PID_PIT.Imax)PID_PIT.Iout=-PID_PIT.Imax;
	}
	//-------------------------------------------------------------------------------------------	
////	PID_YAW.Dout = PID_YAW.D*(Control_proc_yaw_rate - GYRO_Deg.Z);
	PID_YAW.Pout = PID_YAW.P*ctr_angle.yaw;
	PID_YAW.Dout = PID_YAW.D * -GYRO_Deg.Z;
	if(PID_YAW.I!=0)
	{
		PID_YAW.Imax = 50.0f;											//输出限幅在5%油门
		PID_YAW.Iout += PID_YAW.I * ctr_angle.yaw * 0.002f;				//dt = 2ms	
		if(PID_YAW.Iout>PID_YAW.Imax) PID_YAW.Iout=PID_YAW.Imax;						//积分限幅
		if(PID_YAW.Iout<-PID_YAW.Imax)PID_YAW.Iout=-PID_YAW.Imax;
	}
	
	//-------------------------------------------------------------------------------------------

	PID_ROL.ALLout = PID_ROL.Pout + PID_ROL.Dout+ PID_ROL.Iout;
	PID_PIT.ALLout = PID_PIT.Pout + PID_PIT.Dout+ PID_PIT.Iout;
	PID_YAW.ALLout = PID_YAW.Pout + PID_YAW.Dout+ PID_YAW.Iout;
	
	
	roll_out_int = Roll_Rate_Controller(-PID_ROL.ALLout,(BOOL)!throttle);			//角度控制器的输出作为目标角速度，给入速度控制器
	pitch_out_int = Pitch_Rate_Controller(-PID_PIT.ALLout,(BOOL)!throttle);			//(target_rate);当最终油门》0时，禁止i项复位
	
	roll_out_int = constrain_int16(roll_out_int,-LIMITPWM_ROLL,LIMITPWM_ROLL);
	pitch_out_int = constrain_int16(pitch_out_int,-LIMITPWM_PITCH,LIMITPWM_PITCH);
	yaw_out_int = constrain_int16((int16_t)PID_YAW.ALLout,-LIMITPWM_YAW,LIMITPWM_YAW);

	//-------------------------------------------------------------
	
	if(enable==TRUE&&failsafe==TRUE)
	{
			FlyReg.MOTOR1_PWM_REG = throttle + roll_out_int + pitch_out_int - yaw_out_int;
			FlyReg.MOTOR2_PWM_REG = throttle - roll_out_int + pitch_out_int + yaw_out_int;
			FlyReg.MOTOR3_PWM_REG = throttle - roll_out_int - pitch_out_int - yaw_out_int;
			FlyReg.MOTOR4_PWM_REG = throttle + roll_out_int - pitch_out_int + yaw_out_int;
//	
//----------测试用-----------------------------------------------			
//			FlyReg.MOTOR1_PWM_REG = throttle + pitch_out_int;
//			FlyReg.MOTOR2_PWM_REG = throttle + pitch_out_int;
//			FlyReg.MOTOR3_PWM_REG = throttle - pitch_out_int;
//			FlyReg.MOTOR4_PWM_REG = throttle - pitch_out_int;	
//
//			FlyReg.MOTOR1_PWM_REG = throttle + roll_out_int;
//			FlyReg.MOTOR2_PWM_REG = throttle - roll_out_int;
//			FlyReg.MOTOR3_PWM_REG = throttle - roll_out_int;
//			FlyReg.MOTOR4_PWM_REG = throttle + roll_out_int;
//			FlyReg.MOTOR1_PWM_REG = throttle;
//			FlyReg.MOTOR2_PWM_REG = throttle;
//			FlyReg.MOTOR3_PWM_REG = throttle;
//			FlyReg.MOTOR4_PWM_REG = throttle;
//			FlyReg.MOTOR1_PWM_REG = throttle  -	yaw_out_int;
//			FlyReg.MOTOR2_PWM_REG = throttle  + yaw_out_int;
//			FlyReg.MOTOR3_PWM_REG = throttle  - yaw_out_int;
//			FlyReg.MOTOR4_PWM_REG = throttle  + yaw_out_int;		
	}
	else
	{
		FlyReg.MOTOR1_PWM_REG = MOTO_PWM_MIN;
		FlyReg.MOTOR2_PWM_REG = MOTO_PWM_MIN;
		FlyReg.MOTOR3_PWM_REG = MOTO_PWM_MIN;
		FlyReg.MOTOR4_PWM_REG = MOTO_PWM_MIN;
		
		PID_PIT.Iout = 0;
		PID_ROL.Iout = 0;
		PID_YAW.Iout = 0;
		
		Hight_i=0;
	}
	Motor_PwmRflash(FlyReg.MOTOR1_PWM_REG,FlyReg.MOTOR2_PWM_REG,FlyReg.MOTOR3_PWM_REG,FlyReg.MOTOR4_PWM_REG);			
}


/********************************************************************************
 * 函数名：SafeDeal
 * 描述  ：输入角度为实际角度的10倍
 * 输入  ：-	    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
BOOL SafeDeal(s16 rol, s16 pit)	
{
	static s16 MaxAngle = 0;					//用于记录出现的姿态角的最大值
	static s16 MinAngle = 0;					//用于记录出现的姿态角的最小值
	if(MaxAngle<rol)	MaxAngle = rol;
	if(MaxAngle<pit)	MaxAngle = pit;
	if(MinAngle>rol)	MinAngle = rol;
	if(MinAngle>pit)	MinAngle = pit;
	
	if(FlightMode==UNARMED)
	{
		MaxAngle = 0;			//最值清零				
		MinAngle = 0;
		return FALSE;
	}	
	if((FlightMode!=UNARMED)&&((MinAngle<LIMITANGLGE_MIN)||(MaxAngle>LIMITANGLGE_MAX))) 
	{
		return FALSE; 					//飞机可能姿态异常，倾角过大时自动上锁
	}
	return TRUE;
}

/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��AHRS.c
 * ��	��	�����˲ο��㷨
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150612
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.13
**********************************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "AHRS.h"
#include "fly.h"
#include "math.h"
#include "Quaternion.h"
#include "./MPU9250/MPU9250.h"
#include "Attitude.h"

/****************************�궨��***********************************************/
#define FILTERZISE_ACC	10
#define T_SAMPLE		0.002f		//��������
#define FRE_SAMPLE		500			//����Ƶ��
/****************************��������*********************************************/

/****************************��������*********************************************/
ZHOUDATA_f GYRO_Raw,GYRO_Deg,GYRO_OFFSET={-13,15,26};		//��У׼ֵ2000��/s������
ZHOUDATA_f GYRO_Int={0,0,0};								//�����ǲ���������(��̬������)
ZHOUDATA_f ACC_Raw;										//ԭʼ�ļ��ٶ�
ZHOUDATA_f ACC_Last;										//����Ҫʹ�õļ��ٶ�
ZHOUDATA_f ACC_OFFSET={-33,-40,0};//{60,145,-310};		//���ٶȾ�̬ƫ��-+8g������
ZHOUDATA_f ACC_Linear_n;									//�޳����������Լ��ٶ�
ZHOUDATA_f ACC_Linear_n_mm;								//�޳����������Լ��ٶ�,��λmm

ZHOUDATA_f g_LVelimu_n;									//nϵ��imu���Ƶ����ٶ�
ZHOUDATA_f g_LVel_n;										//nϵ�ϵ����ٶ�

s16 ACC_1G = 2042;										//ACC z��⵽��1���������ٶ�

int FiltArray_AX[FILTERZISE_ACC] = {0};
int FiltArray_AY[FILTERZISE_ACC] = {0};
int FiltArray_AZ[FILTERZISE_ACC] = {0};


float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//������
float SinRoll=0,SinPitch=0,CosRoll=0,CosPitch=0,SinYaw,CosYaw;

/****************************��������*********************************************/

/********************************************************************************
 * ��������MPUGyroZeroCal()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
u8 MPUGyroZeroCal()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
	
	GYRO_Raw.X = (s16)FlyReg.IMU_GX_REG;						//������ԭʼ����
	GYRO_Raw.Y = (s16)FlyReg.IMU_GY_REG;
	GYRO_Raw.Z = (s16)FlyReg.IMU_GZ_REG;
	
	if(GYRO_Raw.X>500||GYRO_Raw.X<-500			//pass��ǰ������������� 500Ϊ���Ƶ���ƫ���Χ
	||GYRO_Raw.Y>500||GYRO_Raw.Y<-500
	||GYRO_Raw.Z>500||GYRO_Raw.Z<-500)
	return 1;
	
	tempX += GYRO_Raw.X;
	tempY += GYRO_Raw.Y;
	tempZ += GYRO_Raw.Z;
	calcnt++;
	if(calcnt==1000)								//��1000����ƽ��
	{
		GYRO_OFFSET.X = tempX/calcnt;
		GYRO_OFFSET.Y = tempY/calcnt;
		GYRO_OFFSET.Z = tempZ/calcnt;
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//��λ�������Ա������ĵ���
		return 0;
	}
	else return 1;
}



/********************************************************************************
 * ��������MPUAccZeroCal_GravityMeasure()
 * ����  ��create a quaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
 ********************************************************************************/
u8 MPUAccZeroCal_GravityMeasure()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
//	
	ACC_Raw.X = (s16)FlyReg.IMU_AX_REG;						//������ԭʼ����
	ACC_Raw.Y = (s16)FlyReg.IMU_AY_REG;
	ACC_Raw.Z = (s16)FlyReg.IMU_AZ_REG;
	
	if(ACC_Raw.X>1000||ACC_Raw.X<-1000			//pass��ǰ������������� 1000Ϊ���Ƶ���ƫ���Χ
	||ACC_Raw.Y>1000||ACC_Raw.Y<-1000)
	return 1;
	
	tempX += ACC_Raw.X;
	tempY += ACC_Raw.Y;
	tempZ += ACC_Raw.Z;							//һ�㲻����
	calcnt++;
	if(calcnt==1000)								//��1000����ƽ��
	{
		ACC_OFFSET.X = tempX/calcnt;
		ACC_OFFSET.Y = tempY/calcnt;
		ACC_1G		 = tempZ/calcnt;			//��ǰ����ֵ
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//��λ�������Ա������ĵ���
		return 0;
	}
	else return 1;
}



#define IMU_Kp 0.8f                         //0.8 ֵԽ�󣬶Լ��ٶȵĲο���Խ�� proportional gain governs rate of convergence to accelerometer/magnetometer
#define IMU_Ki 0.001f//0.0003f              // ֵ���˹��Ȼ��й��壬integral gain governs rate of convergence of gyroscope biases
#define IMU_halfT 0.001f//        			// half the sample period,T=2ms

float exInt = 0, eyInt = 0, ezInt = 0;    	// scaled integral error
float exInt_mag = 0, eyInt_mag = 0, ezInt_mag = 0;    	// scaled integral error of mag
/********************************************************************************
 * ��������IMU_update()
 * ����  ��create a quaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
 ********************************************************************************/

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float  wx, wy, wz; 
	float ex, ey, ez;
	
	//���������Ԫ��������任��
	Quaternion_rotation_matrix();				
	
	norm = sqrt(ax*ax + ay*ay + az*az);      	//acc?????
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;								//��һ��ACC������

	// estimated direction of gravity            
	wx = g_MxCnb.a.Z;							//��Ϊ��������ϵz��������ƽ��					
	wy = g_MxCnb.b.Z;
	wz = g_MxCnb.c.Z;

	// error is sum of cross product(������) between reference direction of fields and direction measured by sensors
	ex = (ay*wz - az*wy) ;
	ey = (az*wx - ax*wz) ;
	ez = (ax*wy - ay*wx) ;

	exInt = exInt + ex * IMU_Ki;
	eyInt = eyInt + ey * IMU_Ki;
	ezInt = ezInt + ez * IMU_Ki;

	// adjusted gyroscope measurements
	gx = gx + IMU_Kp*ex + exInt;
	gy = gy + IMU_Kp*ey + eyInt;
	gz = gz + IMU_Kp*ez + ezInt;

	// integrate quaternion rate and normalise						   
	Quat.Q0 = Quat.Q0 + (-Quat.Q1*gx - Quat.Q2*gy - Quat.Q3*gz)*IMU_halfT;
	Quat.Q1 = Quat.Q1 + ( Quat.Q0*gx + Quat.Q2*gz - Quat.Q3*gy)*IMU_halfT;
	Quat.Q2 = Quat.Q2 + ( Quat.Q0*gy - Quat.Q1*gz + Quat.Q3*gx)*IMU_halfT;
	Quat.Q3 = Quat.Q3 + ( Quat.Q0*gz + Quat.Q1*gy - Quat.Q2*gx)*IMU_halfT;

	// normalise quaternion
	norm = sqrt(Quat.Q0*Quat.Q0 + Quat.Q1*Quat.Q1 + Quat.Q2*Quat.Q2 + Quat.Q3*Quat.Q3);

	Quat.Q0 = Quat.Q0 / norm;
	Quat.Q1 = Quat.Q1 / norm;
	Quat.Q2 = Quat.Q2 / norm;
	Quat.Q3 = Quat.Q3 / norm;
}


/********************************************************************************
 * ��������AHRS_SpeedUpdate()
 * ����  ��ʵ��δ�������ٶȸ���,ע����δ���ٶȸ��½��л�����ת����
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/

void AHRS_SpeedUpdate()
{
	ACC_Linear_n.X = ACC_Last.X*g_MxCnb.a.X + ACC_Last.Y*g_MxCnb.b.X + ACC_Last.Z*g_MxCnb.c.X;
	ACC_Linear_n.Y = ACC_Last.X*g_MxCnb.a.Y + ACC_Last.Y*g_MxCnb.b.Y + ACC_Last.Z*g_MxCnb.c.Y;
	ACC_Linear_n.Z = ACC_Last.X*g_MxCnb.a.Z + ACC_Last.Y*g_MxCnb.b.Z + ACC_Last.Z*g_MxCnb.c.Z - ACC_1G;	//����任��Ҫ��ȥ�������ٶ�

	ACC_Linear_n_mm.Z = ACC_Linear_n.Z*Acc_G*1000;
//	g_LVelimu_n.X += ACC_Linear_n.X*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Y += ACC_Linear_n.Y*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Z += ACC_Linear_n.Z*Acc_G*T_AHRS_UPDATE;
}
/********************************************************************************
 * ��������AHRS_Attitude()
 * ����  �����ʲο�ϵͳ������IMUʵ��
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
float v_Yaw_Raw;
void AHRS_Attitude()
{
	u8 i;
	
	static float Yaw_Last= 0,Roll_Last = 0,Pitch_Last= 0;
	static float Yaw_Raw,Roll_Raw,Pitch_Raw;
	
	int tempval1 = 0,tempval2 = 0,tempval3 = 0;
	static u8 filpoint = 0;			//�˲���ָ��
									
//---������----------------------------------------------------------------------------------------------------
	GYRO_Raw.X = (s16)FlyReg.IMU_GX_REG - GYRO_OFFSET.X;						//������ԭʼ����
	GYRO_Raw.Y = (s16)FlyReg.IMU_GY_REG - GYRO_OFFSET.Y;
	GYRO_Raw.Z = (s16)FlyReg.IMU_GZ_REG - GYRO_OFFSET.Z;
	
//	if(GYRO_Raw.X<(-5000)||GYRO_Raw.X>(5000)) GYRO_Raw.X = GYRO_Deg.X; //�������������ʹ��ԭ��ֵ
//	if(GYRO_Raw.Y<(-5000)||GYRO_Raw.Y>(5000)) GYRO_Raw.Y = GYRO_Deg.Y; 
//	if(GYRO_Raw.Z<(-5000)||GYRO_Raw.Z>(5000)) GYRO_Raw.Z = GYRO_Deg.Z; 
	
	GYRO_Deg.X = GYRO_Raw.X * Gyro_G;							//ת���ɶ�/ÿ��
	GYRO_Deg.Y = GYRO_Raw.Y * Gyro_G;
	GYRO_Deg.Z = GYRO_Raw.Z * Gyro_G;

//---���ٶ�----------------------------------------------------------------------------------------------------
	ACC_Raw.X = (s16)FlyReg.IMU_AX_REG - ACC_OFFSET.X;			
	ACC_Raw.Y = (s16)FlyReg.IMU_AY_REG - ACC_OFFSET.Y;
	ACC_Raw.Z = (s16)FlyReg.IMU_AZ_REG;

	FiltArray_AX[filpoint] = (int)ACC_Raw.X;		//ÿ�ɼ�һ������������и������ȵ�ֵ
	FiltArray_AY[filpoint] = (int)ACC_Raw.Y;
	FiltArray_AZ[filpoint] = (int)ACC_Raw.Z;

	for(i=0;i<FILTERZISE_ACC;i++)
	{
		tempval1 += FiltArray_AX[i];
		tempval2 += FiltArray_AY[i];
		tempval3 += FiltArray_AZ[i];
	}
	ACC_Last.X = tempval1/FILTERZISE_ACC;
	ACC_Last.Y = tempval2/FILTERZISE_ACC;
	ACC_Last.Z = tempval3/FILTERZISE_ACC;			//�����ƽ��
	filpoint++;
	if(filpoint==FILTERZISE_ACC) filpoint=0;
	
//---��Ԫ��ˢ����̬----------------------------------------------------------------------------------------------	
	IMU_update(GYRO_Raw.X * Gyro_Gr,GYRO_Raw.Y * Gyro_Gr,GYRO_Raw.Z * Gyro_Gr,
				ACC_Last.X,ACC_Last.Y,ACC_Last.Z);

	//---����������Ƕ�----------------------------------------------------------------------------------------------	
	/*
	// roll����Roll��Pitch��Щ����˵�������෴	�������壺ACC_X��ӦRoll 
	*/	
	Roll_Raw_Rad  = asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2); 
	Pitch_Raw_Rad = atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1); // ptich
//	Yaw_Raw_Rad   = atan2(2 * Quat.Q1 * Quat.Q2 + 2 * Quat.Q0 * Quat.Q3, -2 * Quat.Q2*Quat.Q2 - 2 * Quat.Q3* Quat.Q3 + 1);

//--����������Ǻ���ֵ-------------------------------------------------------
//	CosPitch = cos((double)Pitch_Raw_Rad);
//	SinPitch = sin((double)Pitch_Raw_Rad);				//��roll�ﵽһ���Ƕ�ʱ���������������ʱ���β��á�
//	CosRoll = cos((double)Roll_Raw_Rad);
//	SinRoll = sin((double)Roll_Raw_Rad);

//	Yaw_Raw_Rad = Mag_Yaw_Cal(&Magdecp);				//��λ����
	Yaw_Raw  += GYRO_Deg.Z*T_SAMPLE;
//	if(FlightMode==UNARMED||FlightMode==ARMED)
//	Yaw_Raw = 0;
	
//	Yaw_Raw		= Yaw_Raw_Rad*57.3f;
	Roll_Raw	= Roll_Raw_Rad*57.3f;
	Pitch_Raw	= Pitch_Raw_Rad*57.3f;
	

	if(Roll_Raw<=(-180)||Roll_Raw>=(180))	Roll_Raw = Roll_Last; 
	if(Pitch_Raw<=(-180)||Pitch_Raw>=(180))	Pitch_Raw = Pitch_Last; 
	if(Yaw_Raw>180)	Yaw_Raw = -360.0+Yaw_Raw;
	else if(Yaw_Raw<-180) Yaw_Raw = 360.0+Yaw_Raw;
	
	Roll_Last = Roll_Raw;
	Pitch_Last = Pitch_Raw; 
	Yaw_Last = Yaw_Raw;
	
	FlyReg.ATTI_ROLL_REG = (int16_t)(Roll_Raw*10);					//�Ŵ�10��
	FlyReg.ATTI_PITCH_REG = (int16_t)(Pitch_Raw*10);				//�Ŵ�10��
	FlyReg.ATTI_YAW_REG  = (int16_t)(Yaw_Raw*10);					//�Ŵ�10��
	
	Control_proc_att.roll = Roll_Raw;
	Control_proc_att.pitch= Pitch_Raw;
	Control_proc_att.yaw = Yaw_Raw;
	v_Yaw_Raw = Yaw_Raw;
//	AHRS_SpeedUpdate();
}

#define HP_Par 0.8f
#define LP_Par 0.2f
#define i_Par  0.03f
float errorh_int = 0;
/********************************************************************************
 * ��������AHRS_AltitudeVel()
 * ����  ��������ֱ�ٶ�
 * ����  ��    	
 * ����  ��
 * ����  ��-
 ********************************************************************************/
float AHRS_AltitudeVel(float imuacc_z,float sonarspd)
{
	float tempval = Acc_G*1000*T_ALT_UPDATE;
	float errorh = g_LVel_n.Z - sonarspd;
	float z_rate_n;
	
	errorh_int += i_Par*errorh;						//��������������ٶ����
//	errorh_int = 0;
	if(errorh_int>100) errorh_int = 100;
	if(errorh_int<-100)errorh_int = -100;
	z_rate_n = HP_Par*(g_LVel_n.Z + imuacc_z*tempval) + LP_Par*(sonarspd-errorh_int);
	return z_rate_n;
}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


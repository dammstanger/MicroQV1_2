/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：AHRS.c
 * 描	述	：航姿参考算法
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150612
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.13
**********************************************************************************************/

/****************************包含头文件*******************************************/
#include "AHRS.h"
#include "fly.h"
#include "math.h"
#include "Quaternion.h"
#include "./MPU9250/MPU9250.h"
#include "Attitude.h"

/****************************宏定义***********************************************/
#define FILTERZISE_ACC	10
#define T_SAMPLE		0.002f		//采样周期
#define FRE_SAMPLE		500			//采样频率
/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
ZHOUDATA_f GYRO_Raw,GYRO_Deg,GYRO_OFFSET={-13,15,26};		//简单校准值2000°/s量程下
ZHOUDATA_f GYRO_Int={0,0,0};								//陀螺仪不修正积分(静态测试用)
ZHOUDATA_f ACC_Raw;										//原始的加速度
ZHOUDATA_f ACC_Last;										//最终要使用的加速度
ZHOUDATA_f ACC_OFFSET={-33,-40,0};//{60,145,-310};		//加速度静态偏移-+8g量程下
ZHOUDATA_f ACC_Linear_n;									//剔除重力的线性加速度
ZHOUDATA_f ACC_Linear_n_mm;								//剔除重力的线性加速度,单位mm

ZHOUDATA_f g_LVelimu_n;									//n系上imu估计的线速度
ZHOUDATA_f g_LVel_n;										//n系上的线速度

s16 ACC_1G = 2042;										//ACC z轴测到的1倍重力加速度

int FiltArray_AX[FILTERZISE_ACC] = {0};
int FiltArray_AY[FILTERZISE_ACC] = {0};
int FiltArray_AZ[FILTERZISE_ACC] = {0};


float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//弧度制
float SinRoll=0,SinPitch=0,CosRoll=0,CosPitch=0,SinYaw,CosYaw;

/****************************函数声明*********************************************/

/********************************************************************************
 * 函数名：MPUGyroZeroCal()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
u8 MPUGyroZeroCal()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
	
	GYRO_Raw.X = (s16)FlyReg.IMU_GX_REG;						//陀螺仪原始数据
	GYRO_Raw.Y = (s16)FlyReg.IMU_GY_REG;
	GYRO_Raw.Z = (s16)FlyReg.IMU_GZ_REG;
	
	if(GYRO_Raw.X>500||GYRO_Raw.X<-500			//pass掉前几个错误的数据 500为估计的零偏最大范围
	||GYRO_Raw.Y>500||GYRO_Raw.Y<-500
	||GYRO_Raw.Z>500||GYRO_Raw.Z<-500)
	return 1;
	
	tempX += GYRO_Raw.X;
	tempY += GYRO_Raw.Y;
	tempZ += GYRO_Raw.Z;
	calcnt++;
	if(calcnt==1000)								//加1000次求平均
	{
		GYRO_OFFSET.X = tempX/calcnt;
		GYRO_OFFSET.Y = tempY/calcnt;
		GYRO_OFFSET.Z = tempZ/calcnt;
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//复位参数，以备将来的调用
		return 0;
	}
	else return 1;
}



/********************************************************************************
 * 函数名：MPUAccZeroCal_GravityMeasure()
 * 描述  ：create a quaternion from Euler angles
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
u8 MPUAccZeroCal_GravityMeasure()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
//	
	ACC_Raw.X = (s16)FlyReg.IMU_AX_REG;						//陀螺仪原始数据
	ACC_Raw.Y = (s16)FlyReg.IMU_AY_REG;
	ACC_Raw.Z = (s16)FlyReg.IMU_AZ_REG;
	
	if(ACC_Raw.X>1000||ACC_Raw.X<-1000			//pass掉前几个错误的数据 1000为估计的零偏最大范围
	||ACC_Raw.Y>1000||ACC_Raw.Y<-1000)
	return 1;
	
	tempX += ACC_Raw.X;
	tempY += ACC_Raw.Y;
	tempZ += ACC_Raw.Z;							//一般不矫正
	calcnt++;
	if(calcnt==1000)								//加1000次求平均
	{
		ACC_OFFSET.X = tempX/calcnt;
		ACC_OFFSET.Y = tempY/calcnt;
		ACC_1G		 = tempZ/calcnt;			//当前重力值
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//复位参数，以备将来的调用
		return 0;
	}
	else return 1;
}



#define IMU_Kp 0.8f                         //0.8 值越大，对加速度的参考就越重 proportional gain governs rate of convergence to accelerometer/magnetometer
#define IMU_Ki 0.001f//0.0003f              // 值大了过度会有过冲，integral gain governs rate of convergence of gyroscope biases
#define IMU_halfT 0.001f//        			// half the sample period,T=2ms

float exInt = 0, eyInt = 0, ezInt = 0;    	// scaled integral error
float exInt_mag = 0, eyInt_mag = 0, ezInt_mag = 0;    	// scaled integral error of mag
/********************************************************************************
 * 函数名：IMU_update()
 * 描述  ：create a quaternion from Euler angles
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float  wx, wy, wz; 
	float ex, ey, ez;
	
	//计算基于四元数的坐标变换阵
	Quaternion_rotation_matrix();				
	
	norm = sqrt(ax*ax + ay*ay + az*az);      	//acc?????
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;								//归一化ACC三个轴

	// estimated direction of gravity            
	wx = g_MxCnb.a.Z;							//认为导航坐标系z轴与重力平行					
	wy = g_MxCnb.b.Z;
	wz = g_MxCnb.c.Z;

	// error is sum of cross product(向量积) between reference direction of fields and direction measured by sensors
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
 * 函数名：AHRS_SpeedUpdate()
 * 描述  ：实现未补偿的速度更新,注意暂未对速度更新进行划桨旋转补偿
 * 输入  ：-	    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/

void AHRS_SpeedUpdate()
{
	ACC_Linear_n.X = ACC_Last.X*g_MxCnb.a.X + ACC_Last.Y*g_MxCnb.b.X + ACC_Last.Z*g_MxCnb.c.X;
	ACC_Linear_n.Y = ACC_Last.X*g_MxCnb.a.Y + ACC_Last.Y*g_MxCnb.b.Y + ACC_Last.Z*g_MxCnb.c.Y;
	ACC_Linear_n.Z = ACC_Last.X*g_MxCnb.a.Z + ACC_Last.Y*g_MxCnb.b.Z + ACC_Last.Z*g_MxCnb.c.Z - ACC_1G;	//坐标变换后还要减去重力加速度

	ACC_Linear_n_mm.Z = ACC_Linear_n.Z*Acc_G*1000;
//	g_LVelimu_n.X += ACC_Linear_n.X*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Y += ACC_Linear_n.Y*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Z += ACC_Linear_n.Z*Acc_G*T_AHRS_UPDATE;
}
/********************************************************************************
 * 函数名：AHRS_Attitude()
 * 描述  ：航资参考系统，仅以IMU实现
 * 输入  ：-	    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
float v_Yaw_Raw;
void AHRS_Attitude()
{
	u8 i;
	
	static float Yaw_Last= 0,Roll_Last = 0,Pitch_Last= 0;
	static float Yaw_Raw,Roll_Raw,Pitch_Raw;
	
	int tempval1 = 0,tempval2 = 0,tempval3 = 0;
	static u8 filpoint = 0;			//滤波器指针
									
//---陀螺仪----------------------------------------------------------------------------------------------------
	GYRO_Raw.X = (s16)FlyReg.IMU_GX_REG - GYRO_OFFSET.X;						//陀螺仪原始数据
	GYRO_Raw.Y = (s16)FlyReg.IMU_GY_REG - GYRO_OFFSET.Y;
	GYRO_Raw.Z = (s16)FlyReg.IMU_GZ_REG - GYRO_OFFSET.Z;
	
//	if(GYRO_Raw.X<(-5000)||GYRO_Raw.X>(5000)) GYRO_Raw.X = GYRO_Deg.X; //如果读数有误，则使用原来值
//	if(GYRO_Raw.Y<(-5000)||GYRO_Raw.Y>(5000)) GYRO_Raw.Y = GYRO_Deg.Y; 
//	if(GYRO_Raw.Z<(-5000)||GYRO_Raw.Z>(5000)) GYRO_Raw.Z = GYRO_Deg.Z; 
	
	GYRO_Deg.X = GYRO_Raw.X * Gyro_G;							//转换成度/每秒
	GYRO_Deg.Y = GYRO_Raw.Y * Gyro_G;
	GYRO_Deg.Z = GYRO_Raw.Z * Gyro_G;

//---加速度----------------------------------------------------------------------------------------------------
	ACC_Raw.X = (s16)FlyReg.IMU_AX_REG - ACC_OFFSET.X;			
	ACC_Raw.Y = (s16)FlyReg.IMU_AY_REG - ACC_OFFSET.Y;
	ACC_Raw.Z = (s16)FlyReg.IMU_AZ_REG;

	FiltArray_AX[filpoint] = (int)ACC_Raw.X;		//每采集一个，放入队列中覆盖最先的值
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
	ACC_Last.Z = tempval3/FILTERZISE_ACC;			//求和求平均
	filpoint++;
	if(filpoint==FILTERZISE_ACC) filpoint=0;
	
//---四元数刷新姿态----------------------------------------------------------------------------------------------	
	IMU_update(GYRO_Raw.X * Gyro_Gr,GYRO_Raw.Y * Gyro_Gr,GYRO_Raw.Z * Gyro_Gr,
				ACC_Last.X,ACC_Last.Y,ACC_Last.Z);

	//---横滚、俯仰角度----------------------------------------------------------------------------------------------	
	/*
	// roll这里Roll和Pitch有些书上说的正好相反	这里理清：ACC_X对应Roll 
	*/	
	Roll_Raw_Rad  = asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2); 
	Pitch_Raw_Rad = atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1); // ptich
//	Yaw_Raw_Rad   = atan2(2 * Quat.Q1 * Quat.Q2 + 2 * Quat.Q0 * Quat.Q3, -2 * Quat.Q2*Quat.Q2 - 2 * Quat.Q3* Quat.Q3 + 1);

//--更新相关三角函数值-------------------------------------------------------
//	CosPitch = cos((double)Pitch_Raw_Rad);
//	SinPitch = sin((double)Pitch_Raw_Rad);				//在roll达到一定角度时，会出现死机，暂时屏蔽不用。
//	CosRoll = cos((double)Roll_Raw_Rad);
//	SinRoll = sin((double)Roll_Raw_Rad);

//	Yaw_Raw_Rad = Mag_Yaw_Cal(&Magdecp);				//单位弧度
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
	
	FlyReg.ATTI_ROLL_REG = (int16_t)(Roll_Raw*10);					//放大10倍
	FlyReg.ATTI_PITCH_REG = (int16_t)(Pitch_Raw*10);				//放大10倍
	FlyReg.ATTI_YAW_REG  = (int16_t)(Yaw_Raw*10);					//放大10倍
	
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
 * 函数名：AHRS_AltitudeVel()
 * 描述  ：导航垂直速度
 * 输入  ：    	
 * 返回  ：
 * 调用  ：-
 ********************************************************************************/
float AHRS_AltitudeVel(float imuacc_z,float sonarspd)
{
	float tempval = Acc_G*1000*T_ALT_UPDATE;
	float errorh = g_LVel_n.Z - sonarspd;
	float z_rate_n;
	
	errorh_int += i_Par*errorh;						//加入积分修正加速度误差
//	errorh_int = 0;
	if(errorh_int>100) errorh_int = 100;
	if(errorh_int<-100)errorh_int = -100;
	z_rate_n = HP_Par*(g_LVel_n.Z + imuacc_z*tempval) + LP_Par*(sonarspd-errorh_int);
	return z_rate_n;
}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


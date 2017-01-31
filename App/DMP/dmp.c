#include "dmp.h"
#include "math.h"


s16 Gyro_Offset_X = -74;				//��������ƫ
s16 Gyro_Offset_Y = 5;
s16 Gyro_Offset_Z = 5;

s16 Gyro_Row_X = 0;
s16 Gyro_Row_Y = 0;
s16 Gyro_Row_Z = 0;

s16 Acc_Offset_X = 0;					//���ٶȼ���ƫ
s16 Acc_Offset_Y = 0;
s16 Acc_Offset_Z = -1750;

#define Gyro_toAng 	0.0610351f			//���ٶ�ԭʼ����ת��Ϊ��׼��λ	��/s		gyro/65536*4000
#define Gyro_toRad	0.0010653f			//���ٶ�ԭʼ����ת��Ϊ��׼��λ	����/s		gyro/65535*4000/180*PI

typedef struct 
{
	float Q0;
	float Q1;
	float Q2;
	float Q3;
}Quatdef;

Quatdef Quat={1.0f,0.0f,0.0f,0.0f};
extern uint8_t BUF_T[32];


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);


void AttitudueCaculate(s16 ax, s16 ay, s16 az, s16 gx, s16 gy, s16 gz)
{
	float gx_ang,gy_ang,gz_ang;
static u8 time = 0;
	
	//�����Ǿ�̬�궨/������������ƫ
//	Gyro_Row_X = gx - Gyro_Offset_X;
//	Gyro_Row_Y = gy - Gyro_Offset_Y;
//	Gyro_Row_Z = gz - Gyro_Offset_Z;
	gx -= Gyro_Offset_X;
	gy -= Gyro_Offset_Y;
	gz -= Gyro_Offset_Z;
	FlyReg.IMU_GX_REG = gx;
	//���ٶȼƾ�̬�궨
	ax -= Acc_Offset_X;
	ay -= Acc_Offset_Y;
	az -= Acc_Offset_Z;
	FlyReg.IMU_AZ_REG = az;
	//ת��Ϊ���ʱ�׼��λ
	gx_ang = gx * Gyro_toRad;
	gy_ang = gy * Gyro_toRad;
	gz_ang = gz * Gyro_toRad;
	//�������������ʹ���ϴ�����
	/*
	 * code segment
	 */
	//�˲�
	/*
	 * code segment
	 */
	//��Ԫ��ˢ����̬
	IMUupdate(gx_ang, gy_ang, gz_ang, ax, ay, az);
	//ת��Ϊŷ����
	FlyReg.IMU_AY_REG = (s16)(((-asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2))*57.29578f*100)); 
	FlyReg.IMU_GY_REG = (s16)((atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1))*57.29578f*100); // ptich
//	FlyReg.IMU_GX_REG = (s16)(gx_ang*100);
//	FlyReg.IMU_GY_REG = (s16)(gy_ang*100);
//	Roll_Raw 	  = Roll_Raw_Rad*57.29578f;
//	Pitch_Raw    = Pitch_Raw_Rad*57.29578f;	
	
	
	
	//�������
//	FlyReg.ATTI_PITCH_REG	= gx*100;
//	FlyReg.ATTI_ROLL_REG	= gy*100;
//	FlyReg.ATTI_YAW_REG		= gz*100;
	if (time++>10)
	{
		time=0;
		ImuAckPkgSend();
	}
}


#define Kp 1.0f                        // ֵԽ�󣬶Լ��ٶȵĲο���Խ�� proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0001f//0.0003f             // ֵ���˹��Ȼ��й��壬integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f//0.0025f         // half the sample period???????
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
/******************************************************************************
/ ��������:��Ԫ��ˢ����̬
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
//  float hx, hy, hz, bx, bz;
  float  wx, wy, wz; //vx, vy, vz;//
  float ex, ey, ez;

  // ???????????
  float q0q0 = Quat.Q0*Quat.Q0;
  float q0q1 = Quat.Q0*Quat.Q1;
  float q0q2 = Quat.Q0*Quat.Q2;
//  float q0q3 = Quat.Q0*Quat.Q3;
  float q1q1 = Quat.Q1*Quat.Q1;
//  float q1q2 = Quat.Q1*Quat.Q2;
  float q1q3 = Quat.Q1*Quat.Q3;
  float q2q2 = Quat.Q2*Quat.Q2;
  float q2q3 = Quat.Q2*Quat.Q3;
  float q3q3 = Quat.Q3*Quat.Q3;
	
  if(ax*ay*az==0)
 		return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;							//��һ��ACC������

  // estimated direction of gravity and flux (v and w)             
  wx = 2*(q1q3 - q0q2);												
  wy = 2*(q0q1 + q2q3);
  wz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product(������) between reference direction of fields and direction measured by sensors
  ex = (ay*wz - az*wy) ;                           				
  ey = (az*wx - ax*wz) ;
  ez = (ax*wy - ay*wx) ;

  exInt = exInt + ex * Ki;								  
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							

  // integrate quaternion rate and normalise						   
  Quat.Q0 = Quat.Q0 + (-Quat.Q1*gx - Quat.Q2*gy - Quat.Q3*gz)*halfT;
  Quat.Q1 = Quat.Q1 + (Quat.Q0*gx + Quat.Q2*gz - Quat.Q3*gy)*halfT;
  Quat.Q2 = Quat.Q2 + (Quat.Q0*gy - Quat.Q1*gz + Quat.Q3*gx)*halfT;
  Quat.Q3 = Quat.Q3 + (Quat.Q0*gz + Quat.Q1*gy - Quat.Q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(Quat.Q0*Quat.Q0 + Quat.Q1*Quat.Q1 + Quat.Q2*Quat.Q2 + Quat.Q3*Quat.Q3);
	
  Quat.Q0 = Quat.Q0 / norm;
  Quat.Q1 = Quat.Q1 / norm;
  Quat.Q2 = Quat.Q2 / norm;
  Quat.Q3 = Quat.Q3 / norm;
  
//  SpeedUpdate(ACC_Last.X,ACC_Last.Y,ACC_Last.Z);

}

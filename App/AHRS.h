/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��AHRS.h
 * ��	��	�����˲ο��㷨
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150622
 * ������ϵ	��С����V0.0.2
 * ��汾	��ST3.0.0
 * ����ʱ��	��2015.6.22
 * ���༭	��2015.6.22
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/
#ifndef __AHRS_H_
#define __AHRS_H_

/****************************����ͷ�ļ�*******************************************/
#include "stm32f10x.h"
#include "Quaternion.h"

/****************************�궨��***********************************************/
#define RAD_TO_DEG 				57.295779				//����ת�Ƕ�1rad = 180/��= 57.295779513
#define DEG_TO_RAD    			0.0174533				//�Ƕ�ת����1��= ��/180 =  0.0174532925
//#define Acc_G 		0.0011963f									//���ٶȲ���ֵת����g ��Ӧ��������+-4g  16λADC
#define Acc_G 		0.0382906f										//���ٶȲ���ֵת����m/s2 ��Ӧ��������+-8g  16λADC
#define Gyro_G 		0.0610361f										//�����ǲ���ֵת���ɶ�/ÿ�� �˲�����Ӧ����������2000��ÿ�� 16λADC
#define Gyro_Gr		0.0010653f										//�����ǲ���ֵת���ɻ���/ÿ��
/****************************�ṹ�嶨��*******************************************/

/****************************��������*********************************************/
extern ZHOUDATA_f GYRO_Raw,GYRO_Deg,ACC_Raw,ACC_Last,ACC_OFFSET,GYRO_OFFSET;
extern ZHOUDATA_f GYRO_Int;
extern float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//������

extern ZHOUDATA_f ACC_Linear_n;									//�޳����������Լ��ٶ�
extern ZHOUDATA_f ACC_Linear_n_mm;
extern ZHOUDATA_f g_LVelimu_n;									//nϵ�ϵ����ٶ�
extern ZHOUDATA_f g_LVel_n;										//nϵ�ϵ����ٶ�
extern s16 ACC_1G;
/****************************��������*********************************************/
u8 MPUGyroZeroCal(void);
u8 MPUAccZeroCal_GravityMeasure(void);
void IMU_update(float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_Attitude(void);
float AHRS_AltitudeVel(float imuacc_z,float sonarspd);
extern float v_Yaw_Raw;
#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


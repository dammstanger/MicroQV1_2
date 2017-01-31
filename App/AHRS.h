/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：AHRS.h
 * 描	述	：航姿参考算法
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150622
 * 从属关系	：小四轴V0.0.2
 * 库版本	：ST3.0.0
 * 创建时间	：2015.6.22
 * 最后编辑	：2015.6.22
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
#ifndef __AHRS_H_
#define __AHRS_H_

/****************************包含头文件*******************************************/
#include "stm32f10x.h"
#include "Quaternion.h"

/****************************宏定义***********************************************/
#define RAD_TO_DEG 				57.295779				//弧度转角度1rad = 180/π= 57.295779513
#define DEG_TO_RAD    			0.0174533				//角度转弧度1°= π/180 =  0.0174532925
//#define Acc_G 		0.0011963f									//加速度测量值转换成g 对应传感器的+-4g  16位ADC
#define Acc_G 		0.0382906f										//加速度测量值转换成m/s2 对应传感器的+-8g  16位ADC
#define Gyro_G 		0.0610361f										//陀螺仪测量值转换成度/每秒 此参数对应陀螺仪正负2000度每秒 16位ADC
#define Gyro_Gr		0.0010653f										//陀螺仪测量值转换成弧度/每秒
/****************************结构体定义*******************************************/

/****************************变量声明*********************************************/
extern ZHOUDATA_f GYRO_Raw,GYRO_Deg,ACC_Raw,ACC_Last,ACC_OFFSET,GYRO_OFFSET;
extern ZHOUDATA_f GYRO_Int;
extern float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//弧度制

extern ZHOUDATA_f ACC_Linear_n;									//剔除重力的线性加速度
extern ZHOUDATA_f ACC_Linear_n_mm;
extern ZHOUDATA_f g_LVelimu_n;									//n系上的线速度
extern ZHOUDATA_f g_LVel_n;										//n系上的线速度
extern s16 ACC_1G;
/****************************函数声明*********************************************/
u8 MPUGyroZeroCal(void);
u8 MPUAccZeroCal_GravityMeasure(void);
void IMU_update(float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_Attitude(void);
float AHRS_AltitudeVel(float imuacc_z,float sonarspd);
extern float v_Yaw_Raw;
#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


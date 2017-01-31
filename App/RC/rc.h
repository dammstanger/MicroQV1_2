/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：rc.h
 * 描	述	：
 *                     
 * 实验平台	：ano
 * 硬件连接	：
 * 版 	本	：V1.0.150624
 * 从属关系	：小四轴V0.0.2
 * 库版本	：ST3.0.0
 * 创建时间	：2014.
 * 最后编辑	：2015.6.24
 **-------------------------------------------------------------------------------

 * 作	者	：wk
 * 编辑 者	：dammstanger 
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
#ifndef __RC_H__
#define __RC_H__

/****************************包含头文件*******************************************/
#include "stm32f10x.h"
#include "Project_cfg.h"
#include "fly.h"
#include "hardware.h"
#include "AHRS.h"
#include "DataProcess.h"
#include "debug.h"

extern s16 Moto_PWM_1;
/****************************宏定义***********************************************/

//协议类型	2字节
#define RC_TYPE_L_P				0x00
#define RC_TYPE_H_P				0x01
//协议类型	飞行控制协议
#define RC_TYPE_FLYASK_V		0x8000
#define RC_TYPE_FLYASK_H_V		0x80
#define RC_TYPE_FLYASK_L_V		0x00
//协议类型	飞行控制协议应答
#define RC_TYPE_FLYACK_V		0x8100
#define RC_TYPE_FLYACK_H_V		0x81
#define RC_TYPE_FLYACK_L_V		0x00
//协议类型	惯性器件数据查询协议
#define RC_TYPE_IMUASK_V		0xa000
#define RC_TYPE_IMUASK_H_V		0xa0
#define RC_TYPE_IMUASK_L_V		0x00
//协议类型	惯性器件数据查询协议应答
#define RC_TYPE_IMUACK_V		0xa100
#define RC_TYPE_IMUACK_H_V		0xa1
#define RC_TYPE_IMUACK_L_V		0x00
//协议类型	电机控制协议
#define RC_TYPE_MOTOASK_V		0xc000
#define RC_TYPE_MOTOASK_H_V		0xc0
#define RC_TYPE_MOTOASK_L_V		0x00
//协议类型	电机控制协议应答
#define RC_TYPE_MOTOACK_V		0xc100
#define RC_TYPE_MOTOACK_H_V		0xc1
#define RC_TYPE_MOTOACK_L_V		0x00
//协议类型	寄存器读写协议
#define RC_TYPE_REGWRASK_V		0xe000
#define RC_TYPE_REGWRASK_H_V	0xe0
#define RC_TYPE_REGWRASK_L_V	0x00
//协议类型	寄存器读写协议应答
#define RC_TYPE_REGWRACK_V		0xe100
#define RC_TYPE_REGWRACK_H_V	0xe1
#define RC_TYPE_REGWRACK_L_V	0x00
//协议类型 飞行器姿态查询协议
#define RC_TYPE_ATTIASK_V		0xb000
#define RC_TYPE_ATTIASK_H_V		0xb0
#define RC_TYPE_ATTIASK_L_V		0x00
//协议类型 飞行器姿态查询协议应答
#define RC_TYPE_ATTIACK_V		0xb100
#define RC_TYPE_ATTIACK_H_V		0xb1
#define RC_TYPE_ATTIACK_L_V		0x00

extern __fly_reg *Reg;


#define PkgSend()				\
{								\
	NRF24L01_TxPacket(BUF_T, 10);	\
}


//#define PkgSend()				\
//{								\
//	NRF24L01_Tx();					\
//	NRF24L01_TxPacket(BUF_T, 10);	\
//	NRF24L01_Rx();					\
//}

//飞行控制协议
#define RC_FLYASK_PITCH_L_P		0x02
#define RC_FLYASK_PITCH_H_P		0x03
#define RC_FLYASK_YAW_L_P		0x04
#define RC_FLYASK_YAW_H_P		0x05
#define RC_FLYASK_ROLL_L_P		0x06
#define RC_FLYASK_ROLL_H_P		0x07
#define RC_FLYASK_THRO_L_P		0x08
#define RC_FLYASK_THRO_H_P		0x09
#define RC_FLYASK_CH5_L_P		0x0a
#define RC_FLYASK_CH5_H_P		0x0b
#define RC_FLYASK_CH6_L_P		0x0c
#define RC_FLYASK_CH6_H_P		0x0d
#define IsFlyAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_FLYASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_FLYASK_L_V))

#define FlyAskPkgHandle()		\
{								\
	Reg->FLY_PITCH_REG = ((u16)(*(BUF_R+RC_FLYASK_PITCH_H_P))<<8)|(*(BUF_R+RC_FLYASK_PITCH_L_P));	\
	Reg->FLY_YAW_REG = ((u16)(*(BUF_R+RC_FLYASK_YAW_H_P))<<8)|(*(BUF_R+RC_FLYASK_YAW_L_P));			\
	Reg->FLY_ROLL_REG = ((u16)(*(BUF_R+RC_FLYASK_ROLL_H_P))<<8)|(*(BUF_R+RC_FLYASK_ROLL_L_P));		\
	Reg->FLY_THRO_REG = ((u16)(*(BUF_R+RC_FLYASK_THRO_H_P))<<8)|(*(BUF_R+RC_FLYASK_THRO_L_P));		\
	Reg->FLY_CH5_REG = ((u16)(*(BUF_R+RC_FLYASK_CH5_H_P))<<8)|(*(BUF_R+RC_FLYASK_CH5_L_P));		\
	Reg->FLY_CH6_REG = ((u16)(*(BUF_R+RC_FLYASK_CH6_H_P))<<8)|(*(BUF_R+RC_FLYASK_CH6_L_P));		\
}

#define FlyAckPkgSend()									\
{														\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_FLYASK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_FLYASK_L_V;			\
	*(BUF_T+RC_FLYASK_PITCH_H_P) = Reg->FLY_PITCH_REG>>8;		\
	*(BUF_T+RC_FLYASK_PITCH_L_P) = Reg->FLY_PITCH_REG&0x00ff;	\
	*(BUF_T+RC_FLYASK_YAW_H_P) = Reg->FLY_YAW_REG>>8;		\
	*(BUF_T+RC_FLYASK_YAW_L_P) = Reg->FLY_YAW_REG&0x00ff;	\
	*(BUF_T+RC_FLYASK_ROLL_H_P) = Reg->FLY_ROLL_REG>>8;		\
	*(BUF_T+RC_FLYASK_ROLL_L_P) = Reg->FLY_ROLL_REG&0x00ff;	\
	*(BUF_T+RC_FLYASK_THRO_H_P) = Reg->FLY_THRO_REG>>8;		\
	*(BUF_T+RC_FLYASK_THRO_L_P) = Reg->FLY_THRO_REG&0x00ff;	\
	*(BUF_T+RC_FLYASK_CH5_H_P) = Reg->FLY_CH5_REG>>8;		\
	*(BUF_T+RC_FLYASK_CH5_L_P) = Reg->FLY_CH5_REG&0x00ff;	\
	*(BUF_T+RC_FLYASK_CH6_H_P) = Reg->FLY_CH6_REG>>8;		\
	*(BUF_T+RC_FLYASK_CH6_L_P) = Reg->FLY_CH6_REG&0x00ff;	\
	PkgSend();											\
}

//惯性器件数据查询协议
#define RC_IMUASK_AX_L_P		0x02
#define RC_IMUASK_AX_H_P		0x03
#define RC_IMUASK_AY_L_P		0x04
#define RC_IMUASK_AY_H_P		0x05
#define RC_IMUASK_AZ_L_P		0x06
#define RC_IMUASK_AZ_H_P		0x07
#define RC_IMUASK_GX_L_P		0x08
#define RC_IMUASK_GX_H_P		0x09
#define RC_IMUASK_GY_L_P		0x0a
#define RC_IMUASK_GY_H_P		0x0b
#define RC_IMUASK_GZ_L_P		0x0c
#define RC_IMUASK_GZ_H_P		0x0d

#define IsImuAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_IMUASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_IMUASK_L_V))

#define ImuAskPkgHandle()		\
{								\
}

#define ImuAckPkgSend()			\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_IMUACK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_IMUACK_L_V;			\
	*(BUF_T+RC_IMUASK_AX_H_P) = Reg->IMU_AX_REG>>8;		\
	*(BUF_T+RC_IMUASK_AX_L_P) = Reg->IMU_AX_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_AY_H_P) = Reg->IMU_AY_REG>>8;		\
	*(BUF_T+RC_IMUASK_AY_L_P) = Reg->IMU_AY_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_AZ_H_P) = Reg->IMU_AZ_REG>>8;		\
	*(BUF_T+RC_IMUASK_AZ_L_P) = Reg->IMU_AZ_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GX_H_P) = Reg->IMU_GX_REG>>8;		\
	*(BUF_T+RC_IMUASK_GX_L_P) = Reg->IMU_GX_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GY_H_P) = Reg->IMU_GY_REG>>8;		\
	*(BUF_T+RC_IMUASK_GY_L_P) = Reg->IMU_GY_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GZ_H_P) = Reg->IMU_GZ_REG>>8;		\
	*(BUF_T+RC_IMUASK_GZ_L_P) = Reg->IMU_GZ_REG&0x00ff;	\
	PkgSend();											\
}


//电机控制协议
#define RC_MOTOASK_MOTO1_L_P	0x02
#define RC_MOTOASK_MOTO1_H_P	0x03
#define RC_MOTOASK_MOTO2_L_P	0x04
#define RC_MOTOASK_MOTO2_H_P	0x05
#define RC_MOTOASK_MOTO3_L_P	0x06
#define RC_MOTOASK_MOTO3_H_P	0x07
#define RC_MOTOASK_MOTO4_L_P	0x08
#define RC_MOTOASK_MOTO4_H_P	0x09

#define IsMotoAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_MOTOASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_MOTOASK_L_V))

#define MotoAskPkgHandle()		\
{								\
	Reg->MOTOR1_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO1_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO1_L_P));	\
	Reg->MOTOR2_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO2_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO2_L_P));	\
	Reg->MOTOR3_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO3_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO3_L_P));	\
	Reg->MOTOR4_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO4_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO4_L_P));	\
}

#define MotoAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_MOTOACK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_MOTOACK_L_V;			\
	PkgSend();											\
}

//寄存器读写协议
#define RC_REGWRASK_WRITE_V		0x15
#define RC_REGWRASK_READ_V		0x1a
#define RC_REGWRASK_WR_P		0x02
#define RC_REGWRASK_REG_P		0x03
#define RC_REGWRASK_VAL_L_P		0x04
#define RC_REGWRASK_VAL_H_P		0x05

#define IsRegWrAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_REGWRASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_REGWRASK_L_V))

#define RegWrAskPkgIsWrite()	(*(BUF_R+RC_REGWRASK_WR_P)==RC_REGWRASK_WRITE_V)

#define RegWrAskPkgIsRead()		(*(BUF_R+RC_REGWRASK_WR_P)==RC_REGWRASK_READ_V)

#define RegWritePkgHandle()		\
{								\
	*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))) = ((u16)(*(BUF_R+RC_REGWRASK_VAL_H_P))<<8)|(*(BUF_R+RC_REGWRASK_VAL_L_P));	\
}

#define RegReadPkgHandle()		\
{								\
}

#define RegWrAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_REGWRACK_H_V;		\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_REGWRACK_L_V;		\
	*(BUF_T+RC_REGWRASK_REG_P) = 	*(BUF_R+RC_REGWRASK_REG_P);									\
	*(BUF_T+RC_REGWRASK_VAL_H_P) = 	(*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))))>>8;		\
	*(BUF_T+RC_REGWRASK_VAL_L_P) = 	(*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))))&0x00ff;	\
	PkgSend();											\
}

//飞行器姿态查询协议
#define RC_ATTIASK_PITCH_L_P	0x02
#define RC_ATTIASK_PITCH_H_P	0x03
#define RC_ATTIASK_YAW_L_P		0x04
#define RC_ATTIASK_YAW_H_P		0x05
#define RC_ATTIASK_ROLL_L_P		0x06
#define RC_ATTIASK_ROLL_H_P		0x07
#define RC_IMUASK_AX_L_P		0x02
#define RC_IMUASK_AX_H_P		0x03
#define RC_IMUASK_AY_L_P		0x04
#define RC_IMUASK_AY_H_P		0x05
#define RC_IMUASK_AZ_L_P		0x06
#define RC_IMUASK_AZ_H_P		0x07
#define RC_IMUASK_GX_L_P		0x08
#define RC_IMUASK_GX_H_P		0x09
#define RC_IMUASK_GY_L_P		0x0a
#define RC_IMUASK_GY_H_P		0x0b
#define RC_IMUASK_GZ_L_P		0x0c
#define RC_IMUASK_GZ_H_P		0x0d


#define IsAttiAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_ATTIASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_ATTIASK_L_V))

#define AttiAskPkgHandle()		\
{								\
}

#define AttiAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_ATTIACK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_ATTIACK_L_V;			\
	*(BUF_T+RC_ATTIASK_PITCH_H_P)	= Reg->ATTI_PITCH_REG>>8;		\
	*(BUF_T+RC_ATTIASK_PITCH_L_P)	= Reg->ATTI_PITCH_REG&0x00ff;	\
	*(BUF_T+RC_ATTIASK_YAW_H_P)		= Reg->ATTI_YAW_REG>>8;			\
	*(BUF_T+RC_ATTIASK_YAW_L_P)		= Reg->ATTI_YAW_REG&0x00ff;		\
	*(BUF_T+RC_ATTIASK_ROLL_H_P)	= Reg->ATTI_ROLL_REG>>8;		\
	*(BUF_T+RC_ATTIASK_ROLL_L_P)	= Reg->ATTI_ROLL_REG&0x00ff;	\
	PkgSend();														\
}



//////////////////////////////////////////////////////////////////////

#define IMU_AXOFSET_L_P			0x02
#define IMU_AXOFSET_H_P			0x03
#define IMU_AYOFSET_L_P			0x04
#define IMU_AYOFSET_H_P			0x05
#define IMU_AZOFSET_L_P			0x06
#define IMU_AZOFSET_H_P			0x07
#define IMU_GXDEG_L_P			0x02
#define IMU_GXDEG_H_P			0x03
#define IMU_GYDEG_L_P			0x04
#define IMU_GYDEG_H_P			0x05
#define IMU_GZDEG_L_P			0x06
#define IMU_GZDEG_H_P			0x07


#define DebugAckPkgSend(dat1,dat2,dat3,dat4)	Dug_Pkg_2401_Hunter(BUF_T,dat1,dat2,dat3,dat4);PkgSend()

//	*(BUF_T+RC_ATTIASK_PITCH_H_P)	= Reg->ATTI_PITCH_REG >>8;		\
//	*(BUF_T+RC_ATTIASK_PITCH_L_P)	= Reg->ATTI_PITCH_REG&0x00ff;	\
//	*(BUF_T+IMU_GXDEG_H_P) = (int16_t)(GYRO_Deg.X)>>8;		\
//	*(BUF_T+IMU_GXDEG_L_P) = (int16_t)(GYRO_Deg.X)&0x00ff;	\
//	*(BUF_T+IMU_GYDEG_H_P) = (int16_t)(GYRO_Deg.Y)>>8;		\
//	*(BUF_T+IMU_GYDEG_L_P) = (int16_t)(GYRO_Deg.Y)&0x00ff;	\
//	*(BUF_T+IMU_GZDEG_H_P) = (int16_t)(GYRO_Deg.Z)>>8;		\
//	*(BUF_T+IMU_GZDEG_L_P) = (int16_t)(GYRO_Deg.Z)&0x00ff;	\

//	*(BUF_T+IMU_AXOFSET_H_P)	= (int16_t)GYRO_OFFSET.X>>8;		\
//	*(BUF_T+IMU_AXOFSET_L_P)	= (int16_t)GYRO_OFFSET.X&0x00ff;	\
//	*(BUF_T+IMU_AYOFSET_H_P)	= (int16_t)GYRO_OFFSET.Y>>8;		\
//	*(BUF_T+IMU_AYOFSET_L_P)	= (int16_t)GYRO_OFFSET.Y&0x00ff;	\
//	*(BUF_T+IMU_AZOFSET_H_P)	= (int16_t)GYRO_OFFSET.Z>>8;		\
//	*(BUF_T+IMU_AZOFSET_L_P)	= (int16_t)GYRO_OFFSET.Z&0x00ff;	\
//	*(BUF_T+RC_IMUASK_GX_H_P) = Reg->IMU_GX_REG>>8;		\
//	*(BUF_T+RC_IMUASK_GX_L_P) = Reg->IMU_GX_REG&0x00ff;	\
//	*(BUF_T+RC_IMUASK_GY_H_P) = Reg->IMU_GY_REG>>8;		\
//	*(BUF_T+RC_IMUASK_GY_L_P) = Reg->IMU_GY_REG&0x00ff;	\
//	*(BUF_T+RC_IMUASK_GZ_H_P) = Reg->IMU_GZ_REG>>8;		\
//	*(BUF_T+RC_IMUASK_GZ_L_P) = Reg->IMU_GZ_REG&0x00ff;	\
//	PkgSend();											\
/****************************结构体定义*******************************************/
typedef struct 
{
	u16 ROLL;
	u16 PITCH;
	u16 THROTTLE;
	u16 YAW;
	u16 CH5;
	u16 CH6;
}RC_DATA;

/****************************变量声明*********************************************/
extern RC_DATA Rc_Data;
extern RC_DATA Rc_DataLast;
extern RC_DATA Rc_Orign;
/****************************函数声明*********************************************/

unsigned char RC_Handle(void);
void RCDataProcess(void);
void RC_Dug_pkg(int dat1,int dat2,int dat3,int dat4);


#endif

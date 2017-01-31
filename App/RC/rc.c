#include "rc.h"

/****************************变量定义*********************************************/
uint8_t BUF_R[32] = {0};
uint8_t BUF_T[32] = {0};

__fly_reg *Reg = &FlyReg;


RC_DATA Rc_Data={1500,1500,1000,1500,1000,1000};
				//roll pit throt yaw CH5 CH6
RC_DATA Rc_DataLast={1500,1500,1000,1500,1000,1000};
RC_DATA Rc_Orign={1500,1500,1000,1500,1000,1000};

/******************************************************************************
/ 函数功能:RC数据处理，RC数据范围：1000-2000 
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void RCDataProcess()
{		
	Rc_Data.ROLL =     FlyReg.FLY_ROLL_REG;	//TIM2计数周期为1us，1000刚好1ms,直接赋给
	Rc_Data.PITCH =    FlyReg.FLY_PITCH_REG;
	Rc_Data.THROTTLE = FlyReg.FLY_THRO_REG;
	Rc_Data.YAW =      FlyReg.FLY_YAW_REG;
	Rc_Data.CH5 =      FlyReg.FLY_CH5_REG;
	Rc_Data.CH6 =      FlyReg.FLY_CH6_REG;	

	if(Rc_Data.ROLL>2000&&Rc_Data.ROLL<2100) Rc_Data.ROLL = 2000;		//小范围内超出认为不是错误值，限幅处理
	else if(Rc_Data.ROLL>=2100) Rc_Data.ROLL = Rc_DataLast.ROLL;		//其他情况认为是错误值，保留历史值
	
	if(Rc_Data.PITCH>2000&&Rc_Data.PITCH<2100) Rc_Data.PITCH = 2000;
	else if(Rc_Data.PITCH>=2100)  Rc_Data.PITCH = Rc_DataLast.PITCH;
	
	if(Rc_Data.THROTTLE>2000&&Rc_Data.THROTTLE<2100) Rc_Data.THROTTLE = 2000;
	else if(Rc_Data.THROTTLE>=2100)  Rc_Data.THROTTLE = Rc_DataLast.THROTTLE;
	
	if(Rc_Data.YAW>2000&&Rc_Data.YAW<2100) Rc_Data.YAW = 2000;
	else if(Rc_Data.YAW>=2100)  Rc_Data.YAW = Rc_DataLast.YAW;
	
	if(Rc_Data.CH5<1000||Rc_Data.CH5>2000)     Rc_Data.CH5 = Rc_DataLast.CH5;
	if(Rc_Data.CH6<1000||Rc_Data.CH6>2000)     Rc_Data.CH6 = Rc_DataLast.CH6;
	
	Rc_DataLast.ROLL = Rc_Data.ROLL;
	Rc_DataLast.PITCH = Rc_Data.PITCH;
	Rc_DataLast.THROTTLE = Rc_Data.THROTTLE;
	Rc_DataLast.YAW = Rc_Data.YAW;
	Rc_DataLast.CH5 = Rc_Data.CH5;
	Rc_DataLast.CH6 = Rc_Data.CH6;	
	
}


void RC_Dug_pkg(int dat1,int dat2,int dat3,int dat4)
{
	DebugAckPkgSend(dat1,dat2,dat3,dat4);
}

unsigned char RC_Handle(void)
{
	if(!NRF24L01_RxPacket(BUF_R))		//失败则不解码
	{
		if(IsFlyAskPkg())
		{
			FlyAskPkgHandle();	
	//		FlyAckPkgSend();
			return 1;
		}
		else if(IsImuAskPkg())
		{
			ImuAskPkgHandle();
			ImuAckPkgSend();
		}
		else if(IsMotoAskPkg())
		{
			MotoAskPkgHandle();
			MotoAckPkgSend();
		}
		else if(IsRegWrAskPkg())
		{
			if(RegWrAskPkgIsWrite())
			{
				RegWritePkgHandle();
			}
			else if(RegWrAskPkgIsRead())
			{
				RegReadPkgHandle();
			}
			else
			{
				return 0;
			}
			RegWrAckPkgSend();
			return 1;
		}
		else if(IsAttiAskPkg())
		{
			AttiAskPkgHandle();
			AttiAckPkgSend();
		}
		else
		{
			return 0;
		}
	}
	return 0;
}







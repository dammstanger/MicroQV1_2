#include "app_task.h"

u8	 g_Printcnt = 0;
BOOL g_DataDealOK = FALSE;

OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];
OS_STK task_AttitudeProcess_stk[TASK_AttitudeProcess_STK_SIZE];
OS_STK RC_task_stk[RC_TASK_STK_SIZE];
OS_STK motor_task_stk[MOTOR_TASK_STK_SIZE];
OS_STK Task_LEDState_stk[Task_LEDState_STK_SIZE];


OS_EVENT * msg_nrf;							//NRF信号量事件块指针
OS_EVENT * g_Msgsem_rawdatrdy;				//原始数据采集好
OS_EVENT * g_Msgsem_ctl_att;				//姿态控制的信息量

static void systick_init(void)
{
	SysTick_Config(72000000 / OS_TICKS_PER_SEC);
}


/********************************************************************************
 * 函数名：Delay()
 * 描述  ：软件延时
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
}


void hardware_init(void)
{
	LED_Init();
	USART1_Config(115200);
	Dug_printf("System Init...\r\n");
	LED_Set(1,1);LED_Set(2,1);LED_Set(3,1);LED_Set(4,1);LED_Set(5,1);
	Delay(5000000);
	Motor_Init();
	LED_Set(1,0);LED_Set(2,0);LED_Set(3,0);LED_Set(4,0);LED_Set(5,0);
	Delay(5000000);
	NRF24L01_Init(MODEL_RX, 40);
	LED_Set(1,1);LED_Set(2,1);LED_Set(3,1);LED_Set(4,1);LED_Set(5,1);
	Delay(500000);
	Dug_printf("MPU9250 Init...\r\n");
	MPU9250_Init();
	LED_Set(1,0);LED_Set(2,0);LED_Set(3,0);LED_Set(4,0);LED_Set(5,0);
	Delay(500000);
	LED_Set(1,1);LED_Set(2,1);LED_Set(3,1);LED_Set(4,1);LED_Set(5,1);
	Delay(500000);
	LED_Set(1,0);LED_Set(2,0);LED_Set(3,0);LED_Set(4,0);LED_Set(5,0);
	
	//bsp check
	if (NRF24L01_Check())	{FlyReg.STATE_BSP_REG |= STATE_BSP_NRF;}
//	if (MPU6050_SelfTest())		FlyReg.STATE_BSP_REG |= STATE_BSP_MPU;
	if (FlyReg.STATE_BSP_REG == (STATE_BSP_NRF | STATE_BSP_MPU))		FlyReg.STATE_BSP_REG |= STATE_BSP_ALL_OK;
}


/*
 *	启动任务
 *	系统初始化，创建信号量和其他任务
 */
void startup_task(void *p_arg)
{
	OS_CPU_SR cpu_sr=0;
	
	systick_init();		/* Initialize the SysTick. */
#if (OS_TASK_STAT_EN > 0)
	OSStatInit();		/* Determine CPU capacity. */
#endif
	
	hardware_init();
	PID_Para_Init();
	/* creat message box*/
	msg_nrf 			= 	OSSemCreate(1);			//创建信号量
	g_Msgsem_rawdatrdy 	= 	OSSemCreate(1);			//创建信号量
	g_Msgsem_ctl_att 	= 	OSSemCreate(1);			//创建信号量
	/* TODO: create application tasks here */
 	OS_ENTER_CRITICAL();					//进入临界区  
	OSTaskCreate(Task_AttitudeProcess,(void *)0, 
	&task_AttitudeProcess_stk[TASK_AttitudeProcess_STK_SIZE-1], TASK_AttitudeProcess_PRIO);
	
	OSTaskCreate(RC_task, (void*)0, 
	&RC_task_stk[RC_TASK_STK_SIZE-1], RC_TASK_PRIO);
	
	OSTaskCreate(motor_task, (void*)0, 
	&motor_task_stk[MOTOR_TASK_STK_SIZE-1], MOTOR_TASK_PRIO);
//	
	OSTaskCreate(Task_LEDState, (void*)0, 
	&motor_task_stk[Task_LEDState_STK_SIZE-1], Task_LEDState_PRIO);
//		
	OS_EXIT_CRITICAL();						//退出临界区
	OSTaskDel(OS_PRIO_SELF);
	
	
//	OSTaskSuspend(STARTUP_TASK_PRIO);		//挂起起始任务
}


void Task_AttitudeProcess(void *p_arg)
{	
	static BOOL Gyro_OffsetOK = TRUE;			//已校准
	static BOOL Acc_OffsetOK =  TRUE;
	static u8 Once=1;
	u8 err;
	(void)p_arg;                	
    while (1)
	{
		OSSemPend(g_Msgsem_rawdatrdy, 100, &err);			//等待100ms		
		//SendDebugDat_Hunter(FlyReg.IMU_AX_REG,FlyReg.IMU_AY_REG,FlyReg.IMU_AZ_REG,FlyReg.IMU_GX_REG);
//**************校正部分**********************************************************	
		if(Gyro_OffsetOK==FALSE)					//零偏未校正，则先校正
		{
			if(!MPUGyroZeroCal())					//零偏校正
			{	
				Gyro_OffsetOK = TRUE;		
			}
		}
		if(Acc_OffsetOK==FALSE)						//零偏未校正，则先校正
		{
			if(!MPUAccZeroCal_GravityMeasure())		//加速度零偏校正，和重力测量
			{	
				Acc_OffsetOK = TRUE;
			}
		}

////*******************************************************************************		
		if(Acc_OffsetOK&&Gyro_OffsetOK)
		{
			AttDataProcess();					//耗时900us/4ms	
			FlyModeProcess();
			OSSemPost(g_Msgsem_ctl_att);								
//			SendDebugDat_Hunter(FlyReg.ATTI_ROLL_REG,(int)(-FlyReg.ATTI_ROLL_REG),FlyReg.ATTI_YAW_REG,0);
//			SendDebugDat_Hunter(FlyReg.IMU_AX_REG,FlyReg.IMU_AY_REG,FlyReg.IMU_AZ_REG,FlyReg.IMU_GX_REG);
			if(Once)
			{
				g_DataDealOK = TRUE;
				Once=0;
			}
		}
    }
}


/*
 *	通讯任务
 */
void RC_task(void *p_arg)
{
	u8 err;
	
	while (!(FlyReg.STATE_BSP_REG|STATE_BSP_NRF))		//NRF ERROR
	{
		OSTimeDlyHMSM(0,0,0,500);
	}
	while(1)									//NRF OK
	{
		OSSemPend(msg_nrf, 10, &err);			//等待10*10ms = 100ms
		if (err==OS_ERR_NONE)					//NRF接收到消息
		{
			FlyReg.STATE_SYS_REG |= STATE_SYS_RC_CONNECT;
			RC_Handle();
			RCDataProcess();		//遥控数据处理
			RC_att.roll  = (float)RCdat_T_Angle(Rc_Data.ROLL,(s16)Vehiclebias.X);		//将RC信号转换成角度
			RC_att.pitch = -(float)RCdat_T_Angle(Rc_Data.PITCH,(s16)Vehiclebias.Y);
		}
		else
		{
			FlyReg.STATE_SYS_REG &= ~STATE_SYS_RC_CONNECT;
		}
//		SendDebugDat_Hunter(v_Yaw_Raw,Headhold,v_yaw_err,0);
//		SendDebugDat_Hunter(FlyReg.ATTI_ROLL_REG,FlyReg.ATTI_PITCH_REG,FlyReg.ATTI_YAW_REG,0);
		SendDebugDat_Hunter(FlyReg.IMU_AX_REG,FlyReg.IMU_AY_REG,FlyReg.IMU_AZ_REG,FlyReg.IMU_GX_REG);
//		RC_Dug_pkg(FlyReg.ATTI_ROLL_REG,FlyReg.ATTI_PITCH_REG,FlyReg.ATTI_YAW_REG,0);
//		SendDebugDat_Hunter(GYRO_OFFSET.X,GYRO_OFFSET.Y,GYRO_OFFSET.Z,FlyReg.IMU_AZ_REG);
//		SendDebugDat_Hunter(RC_att.roll,RC_att.pitch,RC_att.yaw,0);
		LEDTrg(1);
	}
}



/*
 *	电机控制任务
 */
void motor_task(void *p_arg)
{
	u8 err;
//	OSTimeDlyHMSM(0,0,5,0);
//	Motor_PwmRflash(throttle,throttle,throttle,throttle);	
//	OSTimeDlyHMSM(0,0,3,0);
//	Motor_PwmRflash(0,0,0,0);

	while(1)
	{
		
		OSSemPend(g_Msgsem_ctl_att, 100, &err);					//等待100ms		
		if (err==OS_ERR_NONE)
		{
			if (FlyReg.CONFIG_REG & MOTOR_EN)
			{			
				CONTROL(Control_ulti_att,0,&FlyReg,TRUE);		
			}
			else
			{
				CONTROL(Control_ulti_att,0,&FlyReg,FALSE);		//PID 依然进行计算只是电机不输出
			}
		}
		
//		OSTimeDlyHMSM(0,0,0,10);
	}
}



//任务1
void Task_LEDState(void *p_arg)
{
	(void)p_arg;                	
	
    while (1)
    {
		if(g_DataDealOK)				//数据处理亮灯一次
		{
			g_DataDealOK = FALSE;
				LED_Set(2,1);
				LED_Set(3,1);
			OSTimeDlyHMSM(0,0,0,500); 
				LED_Set(2,0);
				LED_Set(3,0);	
			OSTimeDlyHMSM(0,0,0,500); 
		}
		else if(g_UART_PIDPkg_Rev_Fin)		//串口接收到PID调试数据
		{
			g_UART_PIDPkg_Rev_Fin = FALSE;
				LED_Set(4,1);
				LED_Set(5,1);
			OSTimeDlyHMSM(0,0,0,500); 
				LED_Set(4,0);
				LED_Set(5,0);	
			OSTimeDlyHMSM(0,0,0,500); 
				LED_Set(4,1);
				LED_Set(5,1);
			OSTimeDlyHMSM(0,0,0,500); 
				LED_Set(4,0);
				LED_Set(5,0);	
			OSTimeDlyHMSM(0,0,0,500); 
		}
		else{
			switch(FlightMode)
			{
				case UNARMED :{
					LED_Set(2,0);
					LED_Set(3,0);
					LED_Set(4,0);
					LED_Set(5,0);
				}break;
				case ARMED :{
					LED_Set(2,1);
					LED_Set(3,1);
					LED_Set(4,0);
					LED_Set(5,0);
				}break;
				case ATTITUDE :{
					LED_Set(2,1);
					LED_Set(3,1);
					LED_Set(4,1);
					LED_Set(5,1);
				}break;
			
				case ATLHOLD :{
	//				LED_Set(2,1);
				}break;
			}
			OSTimeDlyHMSM(0,0,0,100); 
		}
    }
}

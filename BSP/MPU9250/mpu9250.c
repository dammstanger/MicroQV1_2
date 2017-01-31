/****************************************Copyright (c)****************************************************
**                                       辽宁科技大学
**                                     
**                                         电子协会
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           MPU9250.c
** Last modified Date: 
** Last Version: 
** Description:         MPU9250驱动源文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          dammstanger
** Created date:        2016/5/10
** Version:             V1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


/*********************************************************************************************************
	包含头文件
*********************************************************************************************************/
#include "mpu9250.h"
#include "i2c.h"
#include "../uCOS-II/Source/ucos_ii.h"



/*********************************************************************************************************
	预编译处理
*********************************************************************************************************/
#ifdef MPU9250_BUS_I2C
 #ifdef MPU9250_BUS_DEFINED
  #error	//MPU9250总线重复定义
 #endif
 #define MPU9250_BUS_DEFINED
#endif
#ifdef MPU9250_BUS_SPI
 #ifdef MPU9250_BUS_DEFINED
  #error	//MPU9250总线重复定义
 #endif
 #define MPU9250_BUS_DEFINED
 #error		//MPU9250 SPI 总线代码尚未编写
#endif

#ifdef MPU9250_BUS_I2C
 #if MPU9250_AD0 == 0
  #define MPU9250_I2C_ADDR		0xd0
 #elif MPU9250_AD0 == 1
  #define MPU9250_I2C_ADDR		0xd2
 #endif
 #define MPU9250_BufferRead(pBuffer, ReadAddr, NumByteToRead)		 \
					IIC1_MultRead(MPU9250_I2C_ADDR,pBuffer, ReadAddr, NumByteToRead) \
					
 #define MPU9250_BufferWrite(data, WriteAddr)						 \
					IIC1_WriteByte(MPU9250_I2C_ADDR,WriteAddr,data)				 \

//void MPU9250_BufferRead(u8* pBuffer, u8 ReadAddr, u8 NumByteToRead)
//{
//	OS_CPU_SR cpu_sr=0;
//	OS_ENTER_CRITICAL();						
//	I2C_BufferRead(pBuffer, ReadAddr, NumByteToRead);
//	OS_EXIT_CRITICAL();								 //退出临界区
//}

//					
//void MPU9250_BufferWrite(u8 data, u8 WriteAddr)	
//{
//	OS_CPU_SR cpu_sr=0;
//	OS_ENTER_CRITICAL();							 
//	I2C_BufferWrite(data, WriteAddr);
//	OS_EXIT_CRITICAL();								 //退出临界区	
//}

#endif

#ifdef MPU9250_BUS_SPI
 #error		//MPU9250 SPI 总线代码尚未编写
#endif


/*********************************************************************************************************
	常量宏定义--MPU9250内部寄存器地址
*********************************************************************************************************/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，)
#define RA_INT_PIN_CFG  0x37	//中断和管脚配置
#define RA_INT_ENABLE   0x38	//中断使能
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	MPU9250_WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)



/********************************************************************************
 * 函数名：EXTI_PB5_MPU9250_Init()
 * 描述  ：用于配置9250的外部中断
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：外部
 ********************************************************************************/
void EXTI_PB5_MPU9250_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);										//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//使能IRQ所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;					//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 	

}

/*********************************************************************************************************
** Function name:       MPU9250_I2C_Init
** Descriptions:        MPU9250初始化
** input parameters:    none
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void MPU9250_Init(void)
{
	EXTI_PB5_MPU9250_Init();
#ifdef MPU9250_BUS_I2C
	My_I2C_Init(0x0A, MPU9250_I2C_SPEED);		//设置CPU自身I2C地址为0x0a,传输速率为400KHz ;
#endif

#ifdef MPU9250_BUS_SPI
 #error		//MPU9250 SPI 总线代码尚未编写
#endif
	MPU9250_BufferWrite(0x00, PWR_MGMT_1);		//解除休眠状态
	MPU9250_BufferWrite(0x03, CONFIG);			//配置低通滤波器0x06 20160519
	MPU9250_BufferWrite(0x01, SMPLRT_DIV);		//Sample Rate = Gyroscope Output Rate(1kHz) / (1 + SMPLRT_DIV)=500Hz 2ms 20160521
	MPU9250_BufferWrite(0x18, GYRO_CONFIG);		//FS_SEL=3			 ± 2000 °/s
	MPU9250_BufferWrite(0x10, ACCEL_CONFIG);	//AFS_SEL=10b 		 ± 8g
	MPU9250_BufferWrite(0x92, RA_INT_PIN_CFG);	//I2C_BYPASS_EN IIC 直通 /中断为低电平，上拉，自动持续50us，读操作清中断标志
	MPU9250_BufferWrite(0x01, RA_INT_ENABLE);	//使能数据准备好中断
}


/*********************************************************************************************************
** Function name:       MPU9250_SelfTest
** Descriptions:        MPU9250自检
** input parameters:    none
** output parameters:   none
** Returned value:      0:	自检失败		1:自检成功，硬件连接正确
*********************************************************************************************************/
u8 MPU9250_SelfTest(void)
{
	u8 ReadBuf = 0;
	
	MPU9250_BufferRead(&ReadBuf, MPU9250_WHO_AM_I, 1);
	if (ReadBuf == 0x68)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/*********************************************************************************************************
** Function name:       MPU9250_Read
** Descriptions:        MPU9250读取数据，并存入相应寄存器
** input parameters:    none
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
#include "fly.h"
extern __fly_reg FlyReg;
void MPU9250_Read(void)
{
	u8 ReadBuf[14];

	MPU9250_BufferRead(ReadBuf, ACCEL_XOUT_H, 14);
	FlyReg.IMU_AX_REG = (u16)ReadBuf[0]<<8|ReadBuf[1];
	FlyReg.IMU_AY_REG = (u16)ReadBuf[2]<<8|ReadBuf[3];
	FlyReg.IMU_AZ_REG = (u16)ReadBuf[4]<<8|ReadBuf[5];
	FlyReg.IMU_GX_REG = (u16)ReadBuf[8]<<8|ReadBuf[9];
	FlyReg.IMU_GY_REG = (u16)ReadBuf[10]<<8|ReadBuf[11];
	FlyReg.IMU_GZ_REG = (u16)ReadBuf[12]<<8|ReadBuf[13];
}

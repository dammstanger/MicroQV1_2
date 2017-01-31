#ifndef __FLY_H__
#define __FLY_H__


#include "stm32f10x.h"


typedef struct{
uint16_t CONFIG_REG;							/* CONFIG: config quadcopter work mode */
uint16_t STATE_BSP_REG;							/* STATE_BSP: the hardware on board work state */
uint16_t STATE_SYS_REG;							/* STATE_SYS: system work state */
uint16_t STATE_PWR_REG;							/* STATE_PWR: battery level */
uint16_t MOTOR1_PWM_REG;
uint16_t MOTOR2_PWM_REG;
uint16_t MOTOR3_PWM_REG;
uint16_t MOTOR4_PWM_REG;
uint16_t IMU_AX_REG;
uint16_t IMU_AY_REG;
uint16_t IMU_AZ_REG;
uint16_t IMU_GX_REG;
uint16_t IMU_GY_REG;
uint16_t IMU_GZ_REG;
uint16_t FLY_PITCH_REG;							/* 0xf000 is zero,  greater than 0xf000 stand for positive; less than 0xf000 stand for negative */
uint16_t FLY_YAW_REG;
uint16_t FLY_ROLL_REG;
uint16_t FLY_THRO_REG;
uint16_t FLY_CH5_REG;
uint16_t FLY_CH6_REG;
uint16_t PIDPar_CTLTYPE_REG;
uint16_t PIDPar_P_REG;
uint16_t PIDPar_I_REG;
uint16_t PIDPar_D_REG;	
uint16_t PIDPar_IMAX_REG;
int16_t ATTI_PITCH_REG;						/* 0.01¶È */
int16_t ATTI_YAW_REG;
int16_t ATTI_ROLL_REG;
} __fly_reg;
extern __fly_reg FlyReg;


//#define FLY_ADDR				0x1234				/* The quadcopter's address in RC */

/* CONFIG_REG: config/control quadcopter work mode */
#define MOTOR_EN				0x0008				/* motor out enable */
#define CONFIG_LVD				0x0007				/* LVD[2:0] bits (Low Voltage Detector) */
#define CONFIG_LVD_0			0x0001				/* Bits 0 */
#define CONFIG_LVD_1			0x0002				/* Bits 1 */
#define CONFIG_LVD_2			0x0004				/* Bits 2 */

/* STATE_BSP_REG: the hardware on board work state */
#define STATE_BSP_ALL_OK		0x8000				/* BSP all OK bit */
#define STATE_BSP_NRF			0x0001				/* NRF OK bit */
#define STATE_BSP_MPU			0x0002				/* MPU OK bit */

/* STATE_SYS_REG: system work state */
#define STATE_SYS_FLY_STATE		0x0002				/* quadcopter state, flying or landing */
#define STATE_SYS_RC_CONNECT	0x0001				/* 0: Communication disconnected, not received message within 200 ms	*/
													/* 1: Communication connecting, received messages within 200 ms */
#define STATE_SYS_FMODE_UNARMED		0x0000				//UNARMED
#define STATE_SYS_FMODE_ARMED		0x0010				//ARMED
#define STATE_SYS_FMODE_ATTITUDE	0x0020				//ATTITUDE
#define STATE_SYS_FMODE_ATLHOLD		0x0030				//ATLHOLD
#endif

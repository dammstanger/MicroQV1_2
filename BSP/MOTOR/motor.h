#ifndef _BSP_MOTO_H_
#define _BSP_MOTO_H_
#include "stm32f10x.h"

#define Motor_PwmMax 990
//999 为避免电池因过流保护而关闭输出，减小最大值


extern int16_t MOTOR1_PWM;
extern int16_t MOTOR2_PWM;
extern int16_t MOTOR3_PWM;
extern int16_t MOTOR4_PWM;



void Motor_PwmRflash(int16_t motor1_out,int16_t motor2_out,int16_t motor3_out,int16_t motor4_out);
void Motor_Init(void);

#endif

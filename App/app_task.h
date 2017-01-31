#ifndef __APP_TASK__
#define __APP_TASK__

#include "debug.h"
#include "hardware.h"
#include "app_cfg.h"
#include "project_cfg.h"
#include "../uCOS-II/Source/ucos_ii.h"
#include "fly.h"
#include "./RC/rc.h"
#include "AHRS.h"
#include "DataProcess.h"
#include "Control.h"
#include "Attitude.h"
#include "usart.h"
#include "stdio.h"
//#include "./DMP/dmp.h"

extern OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];
extern void startup_task(void *p_arg);

extern OS_STK task_AttitudeProcess_stk[TASK_AttitudeProcess_STK_SIZE];
extern void Task_AttitudeProcess(void *p_arg);

extern OS_STK RC_task_stk[RC_TASK_STK_SIZE];
extern void RC_task(void *p_arg);

extern OS_STK motor_task_stk[MOTOR_TASK_STK_SIZE];
extern void motor_task(void *p_arg);

extern OS_STK Task_LEDState_stk[Task_LEDState_STK_SIZE];
extern void Task_LEDState(void *p_arg);

#endif

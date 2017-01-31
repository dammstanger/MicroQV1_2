/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2006; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                      APPLICATION CONFIGURATION
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.10
* Programmer(s) : BAN
*********************************************************************************************************
*/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__


/* task priority */
#define STARTUP_TASK_PRIO 30
/* task stack size */
#define STARTUP_TASK_STK_SIZE 80

/*attitude process task*/
#define TASK_AttitudeProcess_PRIO 9
#define TASK_AttitudeProcess_STK_SIZE 200
/* nrf task */
#define RC_TASK_PRIO 20
#define RC_TASK_STK_SIZE 80

/* motor task */
#define MOTOR_TASK_PRIO 10
#define MOTOR_TASK_STK_SIZE 80

/*Task_LEDState task*/
#define Task_LEDState_PRIO 29
#define Task_LEDState_STK_SIZE 30
#endif

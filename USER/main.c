#include "ucos_ii.h"
#include "app_task.h"
//#include "sys.h"


int main()
{
	SystemInit();				/* 配置系统时钟为72M */	
	OSInit();
	OSTaskCreate(startup_task, (void*)0, &startup_task_stk[STARTUP_TASK_STK_SIZE - 1], STARTUP_TASK_PRIO);
	OSStart();
	
	return 0;
}

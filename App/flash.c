/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：flach.c
 * 描	述	：外部中断初始化和中断服务
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150412
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.11
 * 最后编辑	：2015.4.12
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#include "flash.h"

/****************************包含头文件*******************************************/

/****************************宏定义***********************************************/


/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
//要写入到STM32 FLASH的字符串数组
u8 TEXT_Buffer[]={"STM32 yyyyyyyyxiaodeng"};
u16 FlashWbuf[FLASH_BUF_SIZE] = {0};
u16 FlashRbuf[FLASH_BUF_SIZE] = {0};

/****************************函数声明*********************************************/



//
u8 Flash_ReadPID(u16 *Par_rol_pit,u16 *Par_yaw,u16 *Par_rp_rate)
{
	u8 i;
	STMFLASH_Read(FLASH_ROOM1_ADDR,FlashRbuf,49);
	
	if(FlashRbuf[0]!=1)
		return (u8)FlashRbuf[0];
	
	for(i=0;i<16;i++)
	{
		Par_rol_pit[i] = FlashRbuf[i+1];
		Par_yaw[i] = FlashRbuf[i+17];
		Par_rp_rate[i] = FlashRbuf[i+33];
	}
	return (u8)FlashRbuf[0];
}



void Flash_WritePID(u16 *Par_rol_pit,u16 *Par_yaw,u16 *Par_rp_rate)
{
	u8 i;
	
	FlashWbuf[0] = 1;
	for(i=0;i<16;i++)
	{
		FlashWbuf[i+1] = Par_rol_pit[i];
		FlashWbuf[i+17] = Par_yaw[i];
		FlashWbuf[i+33] = Par_rp_rate[i];
	}
	
	STMFLASH_Write(FLASH_ROOM1_ADDR,FlashWbuf,49);

}

/******************* (C) COPYRIGHT 2016 DammStanger *****END OF FILE************/

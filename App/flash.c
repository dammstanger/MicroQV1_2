/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��flach.c
 * ��	��	���ⲿ�жϳ�ʼ�����жϷ���
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150412
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.11
 * ���༭	��2015.4.12
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#include "flash.h"

/****************************����ͷ�ļ�*******************************************/

/****************************�궨��***********************************************/


/****************************��������*********************************************/

/****************************��������*********************************************/
//Ҫд�뵽STM32 FLASH���ַ�������
u8 TEXT_Buffer[]={"STM32 yyyyyyyyxiaodeng"};
u16 FlashWbuf[FLASH_BUF_SIZE] = {0};
u16 FlashRbuf[FLASH_BUF_SIZE] = {0};

/****************************��������*********************************************/



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

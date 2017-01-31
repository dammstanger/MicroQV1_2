/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Quaternion.c
 * ��	��	���й���Ԫ��������
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150503
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.5.3
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "Quaternion.h"
#include "math.h"

/****************************�궨��***********************************************/

/****************************��������*********************************************/

/****************************��������*********************************************/
Quatdef Quat={1.0f,0.0f,0.0f,0.0f};

MATRIX_3_3 g_MxCnb;

/****************************��������*********************************************/

/********************************************************************************
 * ��������Euler_to_Quar()
 * ����  ��create a quaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
 ********************************************************************************/
void Euler_to_Quar(float roll, float pitch, float yaw)
{
    float cr2 = cosf(roll*0.5f);
    float cp2 = cosf(pitch*0.5f);
    float cy2 = cosf(yaw*0.5f);
    float sr2 = sinf(roll*0.5f);
    float sp2 = sinf(pitch*0.5f);
    float sy2 = sinf(yaw*0.5f);

    Quat.Q0 = cr2*cp2*cy2 + sr2*sp2*sy2;
    Quat.Q1 = sr2*cp2*cy2 - cr2*sp2*sy2;
    Quat.Q2 = cr2*sp2*cy2 + sr2*cp2*sy2;
    Quat.Q3 = cr2*cp2*sy2 - sr2*sp2*cy2;
}


/********************************************************************************
 * ��������Quar_to_Euler()
 * ����  ����Ԫ��תŷ���ǣ���λ������
 * ����  ������ǵĵ�ַ		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Quar_to_Euler(float *roll,float *pitch,float *yaw)
{
	*pitch  = asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2)* 57.3; // roll
	*roll = atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1)* 57.3; // ptich
	*yaw = 	atan2(2*(Quat.Q1*Quat.Q2 + Quat.Q0*Quat.Q3),Quat.Q0*Quat.Q0+Quat.Q1*Quat.Q1-Quat.Q2*Quat.Q2-Quat.Q3*Quat.Q3) * 57.3;
	
}

/********************************************************************************
 * ��������Quaternion_rotation_matrix
 * ����  ����Ԫ������̬��ת���� ��nϵ��bϵ�����任
 * ����  ��-		    	
 * ����  ��- 
 * ����  ���ⲿ����
 ********************************************************************************/
void Quaternion_rotation_matrix()
{
  float q0q0 = Quat.Q0*Quat.Q0;
  float q0q1 = Quat.Q0*Quat.Q1;
  float q0q2 = Quat.Q0*Quat.Q2;
  float q0q3 = Quat.Q0*Quat.Q3;
  float q1q1 = Quat.Q1*Quat.Q1;
  float q1q2 = Quat.Q1*Quat.Q2;
  float q1q3 = Quat.Q1*Quat.Q3;
  float q2q2 = Quat.Q2*Quat.Q2;
  float q2q3 = Quat.Q2*Quat.Q3;
  float q3q3 = Quat.Q3*Quat.Q3;


	g_MxCnb.a.X = 	q0q0 + q1q1 - q2q2 - q3q3;
	g_MxCnb.a.Y =   2*(q1q2 + q0q3);
	g_MxCnb.a.Z =   2*(q1q3 - q0q2);
	g_MxCnb.b.X =   2*(q1q2 - q0q3);
	g_MxCnb.b.Y = 	q0q0 - q1q1 + q2q2 - q3q3;
	g_MxCnb.b.Z =   2*(q0q1 + q2q3);
	g_MxCnb.c.X =   2*(q1q3 + q0q2);
	g_MxCnb.c.Y =   2*(q2q3 - q0q1);
	g_MxCnb.c.Z = 	q0q0 - q1q1 - q2q2 + q3q3;;
}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/


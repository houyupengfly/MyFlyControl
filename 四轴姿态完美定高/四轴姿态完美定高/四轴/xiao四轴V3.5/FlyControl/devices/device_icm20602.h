/*=============================================================================
** ���ߣ�������
** ���ڣ�2017/10/31
** �汾��V1.0
** ͷ�ļ�
=============================================================================*/


/*============================ INCLUDES ======================================*/
#include "basetype.h"
/*============================ MACROS ========================================*/
#define DISABLE_ICM20602       1
#define ENABLE_ICM20602        0

#define GYRO 					1
#define ACC  					0

#define PWR_MGMT_1				0X6B
#define SIGNAL_PATH_RESET		0X68
#define SMPLRT_DIV				0X19
#define CONFIG					0X1A
#define ACCEL_CONFIG2			0X1D
#define INT_ENABLE				0X38
#define FIFO_EN					0X23
#define INT_PIN_CFG				0X37

#define ACCEL_XOUT_H			0X3B
#define ACCEL_XOUT_L			0X3C
#define ACCEL_YOUT_H			0X3D
#define ACCEL_YOUT_L			0X3E
#define ACCEL_ZOUT_H			0X3F
#define ACCEL_ZOUT_L			0X40

#define TEMP_OUT_H				0X41
#define TEMP_OUT_L				0X42

#define GYRO_XOUT_H				0X43
#define GYRO_XOUT_L				0X44
#define GYRO_YOUT_H				0X45
#define GYRO_YOUT_L				0X46
#define GYRO_ZOUT_H				0X47
#define GYRO_ZOUT_L				0X48

#define TEMP_OUT_H				0X41
#define TEMP_OUT_L				0X42

#define SELF_TEST_X_ACCEL		0X0D	//���ٶ��Լ�Ĵ���X
#define SELF_TEST_Y_ACCEL		0X0E	//���ٶ��Լ�Ĵ���Y
#define SELF_TEST_Z_ACCEL		0X0F	//���ٶ��Լ�Ĵ���Z
#define ACCEL_CONFIG			0X1C	//���ٶ����üĴ���

#define GYRO_CONFIG				0X1B	//���������üĴ���
#define SELF_TEST_X_GYRO		0X50	//�������Լ�Ĵ���X
#define SELF_TEST_Y_GYRO		0X51	//�������Լ�Ĵ���Y
#define SELF_TEST_Z_GYRO		0X52	//�������Լ�Ĵ���Z

#define WHO_AM_I         		0X75	//WHO_AM_I�Ĵ�����ַ
#define ICM20602_WHO_AM_I_CONST (0X12)   //WHO_AM_I����
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
extern u8 ICM_Init(void);
extern void ICM_Get_AccGyroTemp(u8 *buf);
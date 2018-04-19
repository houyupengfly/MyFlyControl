/*=============================================================================
** ���ߣ�������
** ���ڣ�2017/10/31
** �汾��V1.0
** ������:
** �������ܣ�
** ���������
** ��� ��1  ����ֵ��
          2  �ı��ȫ�ֱ���
** ���������õ������������汾�ţ���
** ��ע��
** �޸���־�� �����ߣ����ڣ��޸����ݼ�ԭ��
==============================================================================*/

/*============================ INCLUDES ======================================*/
#include "device_icm20602.h"
#include "device_iic.h"

#include "Fc_systream_time.h"
#include "Fc_data_Struct.h"

/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;

/*----------------------------------------------------------
 + ʵ�ֹ��ܣ�IIC��ȡ1�ֽ�����
 + ���ò������ܣ��豸���Ĵ���������λ��������
----------------------------------------------------------*/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IIC_Read_nByte(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    IIC_Write_1Byte(dev, reg, b) ;
}

/*----------------------------------------------------------
 + ʵ�ֹ��ܣ�IICд��1�ֽ�����
 + ���ò������ܣ��豸���Ĵ�����������ʼλ�����ݳ��ȣ�����
----------------------------------------------------------*/
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b,mask;
    IIC_Read_nByte(dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    IIC_Write_1Byte(dev, reg, b);
}

static void icm20602_setIntEnabled ( void )
{
	IIC_Write_1Byte ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_LEVEL_BIT, ICM_INTMODE_ACTIVEHIGH );
	IIC_Write_1Byte ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_OPEN_BIT, ICM_INTDRV_PUSHPULL );
	IIC_Write_1Byte ( MPUREG_INT_PIN_CFG, ICM_INTCFG_LATCH_INT_EN_BIT, ICM_INTLATCH_50USPULSE);//MPU6050_INTLATCH_WAITCLEAR );
	IIC_Write_1Byte ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_RD_CLEAR_BIT, ICM_INTCLEAR_ANYREAD );

	IIC_Write_1Byte ( MPUREG_INT_ENABLE, ICM_INTERRUPT_DATA_RDY_BIT, 1 );
}

/**************************ʵ�ֺ���********************************************
*��������:	    ��ʼ��icm�������״̬��
*******************************************************************************/
u8 Drv_Icm20602Reg_Init(void)
{
	u8 tmp;
	
    IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_PWR_MGMT_1,0x80);
    Delay_ms(200);
    IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_PWR_MGMT_1,0x01);
    Delay_ms(200);
        
	IIC_Read_1Byte(MPU_IIC_20602,MPUREG_WHOAMI, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	return 0;


	/*��λreg*/
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_SIGNAL_PATH_RESET,0x03);
	Delay_ms(10);
  /*��λreg*/
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_USER_CTRL,0x01);	
	Delay_ms(10);
  
	IIC_Write_1Byte(MPU_IIC_20602,0x70,0x00); 
	Delay_ms(10);
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_PWR_MGMT_2,0x00);
	Delay_ms(10);
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_SMPLRT_DIV,0);
	Delay_ms(10);

	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	Delay_ms(10);
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_GYRO_CONFIG,(3 << 3));
	Delay_ms(10);
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_ACCEL_CONFIG,(2 << 3));
	Delay_ms(10);
	/*���ٶȼ�LPF 20HZ*/
	IIC_Write_1Byte(MPU_IIC_20602,0X1D,0x04);
	Delay_ms(10);
	/*�رյ͹���*/
	IIC_Write_1Byte(MPU_IIC_20602,0X1E,0x00);
	Delay_ms(10);
	/*�ر�FIFO*/
	IIC_Write_1Byte(MPU_IIC_20602,0X23,0x00);
	Delay_ms(10);	
    icm20602_setIntEnabled();
	return 1;

}




void IIC_Icm20602_Read(u8 *buf)
{
	IIC_Read_nByte(MPU_IIC_20602,MPUREG_ACCEL_XOUT_H,14,buf);
    
}




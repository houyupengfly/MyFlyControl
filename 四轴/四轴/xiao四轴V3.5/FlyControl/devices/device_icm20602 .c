/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 函数名:
** 函数功能：
** 输入参数：
** 输出 ：1  返回值：
          2  改变的全局变量
** 本函数调用的其它函数（版本号）：
** 备注：
** 修改日志： （作者，日期，修改内容及原因）
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
 + 实现功能：IIC读取1字节数据
 + 调用参数功能：设备，寄存器，数据位数，数据
----------------------------------------------------------*/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IIC_Read_nByte(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    IIC_Write_1Byte(dev, reg, b) ;
}

/*----------------------------------------------------------
 + 实现功能：IIC写入1字节数据
 + 调用参数功能：设备，寄存器，数据起始位，数据长度，数据
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

/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
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


	/*复位reg*/
	IIC_Write_1Byte(MPU_IIC_20602,MPU_RA_SIGNAL_PATH_RESET,0x03);
	Delay_ms(10);
  /*复位reg*/
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
	/*加速度计LPF 20HZ*/
	IIC_Write_1Byte(MPU_IIC_20602,0X1D,0x04);
	Delay_ms(10);
	/*关闭低功耗*/
	IIC_Write_1Byte(MPU_IIC_20602,0X1E,0x00);
	Delay_ms(10);
	/*关闭FIFO*/
	IIC_Write_1Byte(MPU_IIC_20602,0X23,0x00);
	Delay_ms(10);	
    icm20602_setIntEnabled();
	return 1;

}




void IIC_Icm20602_Read(u8 *buf)
{
	IIC_Read_nByte(MPU_IIC_20602,MPUREG_ACCEL_XOUT_H,14,buf);
    
}




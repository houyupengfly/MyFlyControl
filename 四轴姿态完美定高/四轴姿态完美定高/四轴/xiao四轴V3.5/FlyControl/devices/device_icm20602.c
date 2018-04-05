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
#include "device_spi.h"
#include "stm32f10x.h"
#include "Fc_systream_time.h"
/*============================ MACROS ========================================*/
#define ICM20602_CS_H		GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define ICM20602_CS_L		GPIO_ResetBits(GPIOC, GPIO_Pin_6)
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

void icm20602WriteRegister(u8 reg, u8 value)
{
    SPI2_ReadWriteByte(reg);			    //发送寄存器号，这里可以读取到寄存器的状态
    SPI2_ReadWriteByte(value);      		//写入寄存器的值
}


u8 icm20602ReadRegister(u8 reg)
{
    u8 reg_val;
    //使能SPI传输
    SPI2_ReadWriteByte(reg|0X80);   		//发送寄存器号
    reg_val = SPI2_ReadWriteByte(0XFF);	    //读取寄存器内容，只需要读取，主机可以随意传送数据过去

    return(reg_val);
}

u8 icm20602SpiDetect(void)
{
    u8 tmp;

    tmp=icm20602ReadRegister(WHO_AM_I);
    tmp=icm20602ReadRegister(WHO_AM_I);
    if (tmp == ICM20602_WHO_AM_I_CONST)
    {
        return 1;
    }
    return 0;
}

//设置ICM20602陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:1,设置成功
//    其他,设置失败
void ICM_Set_Gyro_Fsr(u8 fsr)
{
    icm20602WriteRegister(GYRO_CONFIG,fsr<<3);//设置陀螺仪满量程范围
}

//设置ICM20602加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:1,设置成功
//    其他,设置失败
void ICM_Set_Accel_Fsr(u8 fsr)
{
    icm20602WriteRegister(ACCEL_CONFIG,fsr<<3); //设置加速度传感器满量程范围
}

//设置ICM20602的加速度数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:1,设置成功
//    其他,设置失败
void ICM_Acc_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=218)data=7;
    else if(lpf>=99)data=2;
    else if(lpf>=44)data=3;
    else if(lpf>=21)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    icm20602WriteRegister(ACCEL_CONFIG2,data);//设置数字低通滤波器
}


//设置ICM20602的陀螺仪和温度数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:1,设置成功
//    其他,设置失败
void ICM_Gyro_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=250)data=7;
    else if(lpf>=176)data=1;
    else if(lpf>=92)data=2;
    else if(lpf>=41)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    icm20602WriteRegister(CONFIG,data);//设置数字低通滤波器
}

u8 ICM_Set_LPF(u16 lpf)
{
    ICM_Gyro_Set_LPF(lpf/2);	//自动设置LPF为采样率的一半
    ICM_Acc_Set_LPF(lpf/2);
    return 0;
}

void ICM_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    icm20602WriteRegister(SMPLRT_DIV,data);	//设置数字低通滤波器
    ICM_Set_LPF(rate);
}


void ICM_Get_AccGyroTemp(u8 *buf)
{
    buf[0]=icm20602ReadRegister(ACCEL_XOUT_H);
    buf[1]=icm20602ReadRegister(ACCEL_XOUT_H+1);
    buf[2]=icm20602ReadRegister(ACCEL_XOUT_H+2);
    buf[3]=icm20602ReadRegister(ACCEL_XOUT_H+3);
    buf[4]=icm20602ReadRegister(ACCEL_XOUT_H+4);
    buf[5]=icm20602ReadRegister(ACCEL_XOUT_H+5);
    buf[6]=icm20602ReadRegister(ACCEL_XOUT_H+6);
    buf[7]=icm20602ReadRegister(ACCEL_XOUT_H+7);
    buf[8]=icm20602ReadRegister(ACCEL_XOUT_H+8);
    buf[9]=icm20602ReadRegister(ACCEL_XOUT_H+9);
    buf[10]=icm20602ReadRegister(ACCEL_XOUT_H+10);
    buf[11]=icm20602ReadRegister(ACCEL_XOUT_H+11);

}

u8 ICM_Init(void)
{
    return icm20602SpiDetect();					//器件ID不正确,返回false
    icm20602WriteRegister(PWR_MGMT_1,0X80);	//复位icm20602
    Delay_ms(100);
    icm20602WriteRegister(SIGNAL_PATH_RESET,0X03); //Reset accel digital signal path. Reset temp digital signal path.
    Delay_ms(100);
//	icm20602WriteRegister(PWR_MGMT_1,0X00); //唤醒icm20602
    ICM_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
    Delay_ms(15);
    ICM_Set_Accel_Fsr(0);					//加速度传感器,±2g
    Delay_ms(15);
    ICM_Set_Rate(50);						//设置采样率50Hz
    Delay_ms(15);
    icm20602WriteRegister(INT_PIN_CFG,0X10);
    Delay_ms(15);
    icm20602WriteRegister(INT_ENABLE,0X00);	//关闭所有中断


}





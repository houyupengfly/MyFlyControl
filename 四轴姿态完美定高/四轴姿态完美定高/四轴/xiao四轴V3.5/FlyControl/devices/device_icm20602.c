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
    SPI2_ReadWriteByte(reg);			    //���ͼĴ����ţ�������Զ�ȡ���Ĵ�����״̬
    SPI2_ReadWriteByte(value);      		//д��Ĵ�����ֵ
}


u8 icm20602ReadRegister(u8 reg)
{
    u8 reg_val;
    //ʹ��SPI����
    SPI2_ReadWriteByte(reg|0X80);   		//���ͼĴ�����
    reg_val = SPI2_ReadWriteByte(0XFF);	    //��ȡ�Ĵ������ݣ�ֻ��Ҫ��ȡ�������������⴫�����ݹ�ȥ

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

//����ICM20602�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:1,���óɹ�
//    ����,����ʧ��
void ICM_Set_Gyro_Fsr(u8 fsr)
{
    icm20602WriteRegister(GYRO_CONFIG,fsr<<3);//���������������̷�Χ
}

//����ICM20602���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:1,���óɹ�
//    ����,����ʧ��
void ICM_Set_Accel_Fsr(u8 fsr)
{
    icm20602WriteRegister(ACCEL_CONFIG,fsr<<3); //���ü��ٶȴ����������̷�Χ
}

//����ICM20602�ļ��ٶ����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:1,���óɹ�
//    ����,����ʧ��
void ICM_Acc_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=218)data=7;
    else if(lpf>=99)data=2;
    else if(lpf>=44)data=3;
    else if(lpf>=21)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    icm20602WriteRegister(ACCEL_CONFIG2,data);//�������ֵ�ͨ�˲���
}


//����ICM20602�������Ǻ��¶����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:1,���óɹ�
//    ����,����ʧ��
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
    icm20602WriteRegister(CONFIG,data);//�������ֵ�ͨ�˲���
}

u8 ICM_Set_LPF(u16 lpf)
{
    ICM_Gyro_Set_LPF(lpf/2);	//�Զ�����LPFΪ�����ʵ�һ��
    ICM_Acc_Set_LPF(lpf/2);
    return 0;
}

void ICM_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    icm20602WriteRegister(SMPLRT_DIV,data);	//�������ֵ�ͨ�˲���
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
    return icm20602SpiDetect();					//����ID����ȷ,����false
    icm20602WriteRegister(PWR_MGMT_1,0X80);	//��λicm20602
    Delay_ms(100);
    icm20602WriteRegister(SIGNAL_PATH_RESET,0X03); //Reset accel digital signal path. Reset temp digital signal path.
    Delay_ms(100);
//	icm20602WriteRegister(PWR_MGMT_1,0X00); //����icm20602
    ICM_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
    Delay_ms(15);
    ICM_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
    Delay_ms(15);
    ICM_Set_Rate(50);						//���ò�����50Hz
    Delay_ms(15);
    icm20602WriteRegister(INT_PIN_CFG,0X10);
    Delay_ms(15);
    icm20602WriteRegister(INT_ENABLE,0X00);	//�ر������ж�


}






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
#include "device_mpc2520.h"
#include "device_iic.h"

#include "Fc_systream_time.h"
#include "Fc_seedEstimated.h"
#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
/*============================ STATIC VARIABLES ==============================*/
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;
float baro_Offset,alt_3,height;
u8 hard_error_mpc2520;
/*============================ FUNCTION ======================================*/
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
    hard_error_mpc2520=IIC_Write_1Byte(hwadr,regadr,val);
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 Spl0601_Read(unsigned char hwadr, unsigned char regadr)
{
    u8 reg_data;

    IIC_Read_1Byte(hwadr,regadr,&reg_data);
    return reg_data;
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/

void Spl0601_Rateset(u8 iSensor, u8 u8SmplRate, u8 u8OverSmpl)
{
    u8 reg = 0;
    s32 i32kPkT = 0;
    switch(u8SmplRate)
    {
    case 2:
        reg |= (1<<5);
        break;
    case 4:
        reg |= (2<<5);
        break;
    case 8:
        reg |= (3<<5);
        break;
    case 16:
        reg |= (4<<5);
        break;
    case 32:
        reg |= (5<<5);
        break;
    case 64:
        reg |= (6<<5);
        break;
    case 128:
        reg |= (7<<5);
        break;
    case 1:
    default:
        break;
    }
    switch(u8OverSmpl)
    {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096;
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }

    if(iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        Spl0601_Write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = Spl0601_Read(HW_ADR, 0x09);
            Spl0601_Write(HW_ADR, 0x09, reg | 0x04);
        }
    }
    if(iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        Spl0601_Write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = Spl0601_Read(HW_ADR, 0x09);
            Spl0601_Write(HW_ADR, 0x09, reg | 0x08);
        }
    }

}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/

void Spl0601_Get_Calib_Param(void)
{
    u32 h;
    u32 m;
    u32 l;
    h =  Spl0601_Read(HW_ADR, 0x10);
    l  =  Spl0601_Read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (s16)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    h =  Spl0601_Read(HW_ADR, 0x11);
    l  =  Spl0601_Read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (s16)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    h =  Spl0601_Read(HW_ADR, 0x13);
    m =  Spl0601_Read(HW_ADR, 0x14);
    l =  Spl0601_Read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (s32)h<<12 | (s32)m<<4 | (s32)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    h =  Spl0601_Read(HW_ADR, 0x15);
    m =  Spl0601_Read(HW_ADR, 0x16);
    l =  Spl0601_Read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (s32)h<<16 | (s32)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    h =  Spl0601_Read(HW_ADR, 0x18);
    l  =  Spl0601_Read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (s16)h<<8 | l;
    h =  Spl0601_Read(HW_ADR, 0x1A);
    l  =  Spl0601_Read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (s16)h<<8 | l;
    h =  Spl0601_Read(HW_ADR, 0x1C);
    l  =  Spl0601_Read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (s16)h<<8 | l;
    h =  Spl0601_Read(HW_ADR, 0x1E);
    l  =  Spl0601_Read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (s16)h<<8 | l;
    h =  Spl0601_Read(HW_ADR, 0x20);
    l  =  Spl0601_Read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (s16)h<<8 | l;
}



/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/

void Fc_Spl0601_Init(void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = 0x34;
    Spl0601_Get_Calib_Param();


    Spl0601_Rateset(PRESSURE_SENSOR,128, 16);
    // sampling rate = 1Hz; Temperature oversample = 1;
    Spl0601_Rateset(TEMPERATURE_SENSOR,8, 8);

    //Start background measurement
    Spl0601_Start_Continuous(CONTINUOUS_P_AND_T);

}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Start_Temperature(void)
{
    Spl0601_Write(HW_ADR, 0x08, 0x02);
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Start_Pressure(void)
{
    Spl0601_Write(HW_ADR, 0x08, 0x01);
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Start_Continuous(u8 mode)
{
    Spl0601_Write(HW_ADR, 0x08, mode+4);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Get_Raw_Temp(void)
{
    u8 h[3] = {0};

    h[0] = Spl0601_Read(HW_ADR, 0x03);
    h[1] = Spl0601_Read(HW_ADR, 0x04);
    h[2] = Spl0601_Read(HW_ADR, 0x05);

    p_spl0601->i32rawTemperature = (s32)h[0]<<16 | (s32)h[1]<<8 | (s32)h[2];
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Spl0601_Get_Raw_Pressure(void)
{
    u8 h[3];

    h[0] = Spl0601_Read(HW_ADR, 0x00);
    h[1] = Spl0601_Read(HW_ADR, 0x01);
    h[2] = Spl0601_Read(HW_ADR, 0x02);

    p_spl0601->i32rawPressure = (s32)h[0]<<16 | (s32)h[1]<<8 | (s32)h[2];
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}


/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
float Spl0601_Get_Temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/

float Spl0601_Get_Pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

float user_spl0601_get(void)
{
    float baro_pressure,alt_high;

    Spl0601_Get_Raw_Temp();
    Spl0601_Get_Raw_Pressure();
    baro_pressure = Spl0601_Get_Pressure();
    g_FlyControlDataStruct.AppBaroDataStruct.Pruess=baro_pressure;
    alt_3 = ( 101000 - baro_pressure ) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101000 - baro_pressure ) * 100.0f ;

    alt_high = ( height - baro_Offset) ;

    //气压高度校准
    if( g_FlyControlDataStruct.AppHightCtrl.baroStart == 1)
    {
        g_FlyControlDataStruct.AppHightCtrl.baroStart = 0;
        alt_high = 0;
        baro_Offset = height;
        Fc_Alti_Estimate_Reset();

    }
    g_FlyControlDataStruct.AppConfigDataStruct.BaroFlag =Baro_State_Ready;
    return g_FlyControlDataStruct.AppHightCtrl.BaroHight= alt_high;

}



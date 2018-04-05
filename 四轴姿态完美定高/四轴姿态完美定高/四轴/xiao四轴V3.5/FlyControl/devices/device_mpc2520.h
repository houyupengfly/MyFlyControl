/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/

#ifndef MPC2520_01_H
#define MPC2520_01_H
/*============================ INCLUDES ======================================*/
#include <stm32f10x.h>
#include "basetype.h"
/*============================ MACROS ========================================*/
#define HW_ADR 0x76 //0x76 //0x77 01  0x76  00   7bit  0xec
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


/*============================ TYPES =========================================*/

struct spl0601_calib_param_t {
    s16 c0;
    s16 c1;
    s32 c00;
    s32 c10;
    s16 c01;
    s16 c11;
    s16 c20;
    s16 c21;
    s16 c30;
};

struct spl0601_t {
    struct spl0601_calib_param_t calib_param;/**<calibration data*/
    u8 chip_id; /**<chip id*/
    s32 i32rawPressure;
    s32 i32rawTemperature;
    s32 i32kP;
    s32 i32kT;
};
/*============================ GLOBAL VARIABLES ==============================*/

extern float temperature,temperature2;

/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

extern void Fc_Spl0601_Init(void);
extern void Spl0601_Rateset(u8 iSensor, u8 u8OverSmpl, u8 u8SmplRate);
extern void Sexternpl0601_Start_Temperature(void);
extern void Spl0601_Start_Pressure(void);
extern void Spl0601_Start_Continuous(u8 mode);
extern void Spl0601_Get_Raw_Temp(void);
extern void Spl0601_Get_Raw_Pressure(void);
extern float Spl0601_Get_Temperature(void);
extern float Spl0601_Get_Pressure(void);
extern float user_spl0601_get(void);
#endif


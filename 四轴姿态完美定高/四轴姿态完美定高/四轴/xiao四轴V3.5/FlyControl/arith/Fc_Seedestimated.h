/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/
#ifndef __SPEEDESTIMATD_H__
#define __SPEEDESTIMATD_H__  
/*============================ INCLUDES ======================================*/
#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
extern void SpeedEstimated(void);
extern void Strapdown_INS_Reset(SINS *Ins,u8 Axis,float Pos_Target,float Vel_Target);
extern void Fc_Hight_Get(float T);
extern void Fc_Alti_Estimate(u8 T);
extern void Fc_Alti_Estimate_Reset(void);
extern void Fc_Alti_Estimatepx4(void);
extern void SINS_Z(float dt);
#endif

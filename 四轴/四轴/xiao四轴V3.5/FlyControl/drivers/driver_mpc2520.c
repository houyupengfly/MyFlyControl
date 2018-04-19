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
#include "driver_mpc2520.h"
#include "device_mpc2520.h"

#include "Fc_seedEstimated.h"
#include "Fc_data_Struct.h"

#include <math.h>
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/

extern FlyControlDataStruct g_FlyControlDataStruct;
extern SINS NewSinsData;
extern  float z_est[3];//! z轴，1.估算高度; 2.速度; 3.加速度
/*============================ STATIC VARIABLES ==============================*/

unsigned int spl06001_Cnt=0;

float spl06001_Filter_P,spl06001_Filter_high;
float temperature;
float pressure;
float spl06001_Offset=0;
float true_altitude=0;   //实际高度,单位:m
float temp_baro;

u8 spl06001_Offset_Okay=0;
u8 Spl06001_Finished=0;

/*============================ FUNCTION ======================================*/

 /*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float User_Spl0601_Get()
{
    static uint16_t spl06_cnt=0;
    spl06_cnt++;
    if(spl06_cnt==1)
    {
        Spl0601_Get_Raw_Temp();
        temperature = Spl0601_Get_Temperature();
        g_FlyControlDataStruct.AppBaroDataStruct.Temp = temperature;
    }
    else
    {

        Spl0601_Get_Raw_Pressure();
        pressure = Spl0601_Get_Pressure();
        g_FlyControlDataStruct.AppBaroDataStruct.Pruess = pressure;
        spl06_cnt=0;
    }
    return 0;
}

 /*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/

float Get_Altitude_SPL06_001(float baroPress)
{
    float Tempbaro=(float)(baroPress/spl06001_Offset)*1.0;
    true_altitude = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));
    return true_altitude;
}
 /*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/

void Fc_Baro_Get(void)
{
    //获取气压和温度
//    User_Spl0601_Get();
//    //前50次数据不要 
//    if(spl06001_Cnt<=50)  
//    {
//        spl06001_Cnt++;
//    }
//    //气压计状态可用
//    if(spl06001_Cnt==49)
//    {
//        spl06001_Offset_Okay=1;
//        g_FlyControlDataStruct.AppConfigDataStruct.BaroFlag = Baro_State_Ready;
//        spl06001_Offset=pressure;
//    }
//    
//    if(spl06001_Offset_Okay==1)//初始气压零点设置完毕
//    {
//        spl06001_Filter_P=pressure;
//        g_FlyControlDataStruct.AppHightCtrl.BaroHight=Get_Altitude_SPL06_001(spl06001_Filter_P);
//    }
//    //当需要校准气压的时候 一定要清除掉惯性估计   
//    if( g_FlyControlDataStruct.AppHightCtrl.baroStart == 1)
//    {
//        g_FlyControlDataStruct.AppHightCtrl.baroStart = 0;
//        g_FlyControlDataStruct.AppHightCtrl.BaroHight = 0;
//        spl06001_Offset=pressure;
//        z_est[0]=0;
//        z_est[1]=0;
//        z_est[2]=0;
////        Strapdown_INS_Reset(&NewSinsData,0,g_FlyControlDataStruct.AppHightCtrl.BaroHight  ,0);
        
//    }	
 

}


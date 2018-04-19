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
extern  float z_est[3];//! z�ᣬ1.����߶�; 2.�ٶ�; 3.���ٶ�
/*============================ STATIC VARIABLES ==============================*/

unsigned int spl06001_Cnt=0;

float spl06001_Filter_P,spl06001_Filter_high;
float temperature;
float pressure;
float spl06001_Offset=0;
float true_altitude=0;   //ʵ�ʸ߶�,��λ:m
float temp_baro;

u8 spl06001_Offset_Okay=0;
u8 Spl06001_Finished=0;

/*============================ FUNCTION ======================================*/

 /*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
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
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/

float Get_Altitude_SPL06_001(float baroPress)
{
    float Tempbaro=(float)(baroPress/spl06001_Offset)*1.0;
    true_altitude = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));
    return true_altitude;
}
 /*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/

void Fc_Baro_Get(void)
{
    //��ȡ��ѹ���¶�
//    User_Spl0601_Get();
//    //ǰ50�����ݲ�Ҫ 
//    if(spl06001_Cnt<=50)  
//    {
//        spl06001_Cnt++;
//    }
//    //��ѹ��״̬����
//    if(spl06001_Cnt==49)
//    {
//        spl06001_Offset_Okay=1;
//        g_FlyControlDataStruct.AppConfigDataStruct.BaroFlag = Baro_State_Ready;
//        spl06001_Offset=pressure;
//    }
//    
//    if(spl06001_Offset_Okay==1)//��ʼ��ѹ����������
//    {
//        spl06001_Filter_P=pressure;
//        g_FlyControlDataStruct.AppHightCtrl.BaroHight=Get_Altitude_SPL06_001(spl06001_Filter_P);
//    }
//    //����ҪУ׼��ѹ��ʱ�� һ��Ҫ��������Թ���   
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


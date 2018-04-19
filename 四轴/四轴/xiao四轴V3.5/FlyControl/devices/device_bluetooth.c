/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/


/*============================ INCLUDES ======================================*/

#include "device_Bluetooth.h"
#include "device_Uart.h"

#include "Fc_Data_Transfer.h"
#include "Fc_data_Struct.h"

/*============================ MACROS ========================================*/
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
/*============================ STATIC VARIABLES ==============================*/
static u8 RxBuffer[50];
u16 Bluet_Erro;
u8 Bluet_SSI,Bluet_SSI_CNT;
/*============================ FUNCTION ======================================*/


/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Bluet_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0;
    for(u8 i=0; i<(num-1); i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))		return;
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;

    if(*(data_buf+2)==0X01)
    {
        if(*(data_buf+4)==0XA2)//一键降落
        {
            if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==1)
            {
                g_FlyControlDataStruct.AppConfigDataStruct.LandDown = 1;
            }
        }
        else if(*(data_buf+4)==0XA3)//一键起飞
        {
            if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==0)
            {
                g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 1;
                g_FlyControlDataStruct.AppHightCtrl.StartHight=80;
            }
        }
        else if(*(data_buf+4)==0XA0)//停机
        {
            if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==1)
            {
                g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 0;
            }
        }\
        else if(*(data_buf+4)==0XA5)//定高模式
        {
            g_FlyControlDataStruct.AppConfigDataStruct.FlyMode = 1;
        }
        else if(*(data_buf+4)==0XA4)//普通模式
        {
            g_FlyControlDataStruct.AppConfigDataStruct.FlyMode = 0;
        }
        else if(*(data_buf+4)==0XA1)//抛飞模式
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Throw= 1;
        }
    }

    if(*(data_buf+2)==0X03)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.NS = 3;	//遥控数据来源（1：遥控器,3：蓝牙模块）

        g_FlyControlDataStruct.AppRcDataStruct.THR = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
        g_FlyControlDataStruct.AppRcDataStruct.Yaw = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
        g_FlyControlDataStruct.AppRcDataStruct.Roll = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
        g_FlyControlDataStruct.AppRcDataStruct.Pitch = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
        g_FlyControlDataStruct.AppRcDataStruct.AUX1= (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
        g_FlyControlDataStruct.AppRcDataStruct.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
    }
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Bluet_Receive_Prepare(u8 data)
{
    static u8 _data_len = 0,_data_cnt = 0;
    static u8 state = 0;

    if(state==0&&data==0xAA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF)
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2&&data<0XF1)
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        Bluet_Erro = 0;

    }
    else
        state = 0;
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_Blue_Init(void)
{
    Fc_Uart4_Init(9600);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 Bluet_Connect(void)
{
    static u8 Connect_flag;

    Bluet_Erro ++;
    if(Bluet_Erro==1)
    {
        Bluet_SSI_CNT++;
        Bluet_Receive_Anl(RxBuffer,RxBuffer[3]+5);
        Connect_flag = 1;
    }
    if(Bluet_Erro>=500)
    {
        Bluet_Erro = 1;
        Connect_flag = 0;
    }
    return Connect_flag;
}

/**
* @file     App_RC.c
* @brief    ң������Ӧ�÷�����Ŀ�����
* @version  V2.0
* @author   HYP
* @date     2017-4-27
* @note
*/
/*============================ INCLUDES ======================================*/

#include "device_Nrf24l01.h"
#include "device_Bluetooth.h"
#include "device_flow.h"

#include "Fc_revice.h"
#include "Fc_data_Struct.h"
#include <math.h>
#include "mymath.h"

/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/



/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
extern u8 Bluet_SSI,Bluet_SSI_CNT;
/*============================ STATIC VARIABLES ==============================*/
u8 unlook,look,Calibrate_cnt=0;
u16 test_flag,set_flag;

float Rc_Angle,Rc_Abs;
/*============================ IMPLEMENTATION ================================*/
/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_RadioContrl(void)
{
    Nrf_Check_Event();//NRF
    if(Nrf_Connect()==0)
    {
        Bluet_Connect();
    }
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_NoHeadMode_Establishment(float *x,float *y,float angle)
{
    angle    = angle / ANGLETORAD;

    if(Rc_Abs==0)        //ת��Ƕ�ת����
    {
        Rc_Angle=0;
    }
    else
    {
        Rc_Angle = asin( (*x)/Rc_Abs );                  //ң�˽Ƕ�
    }
    //�Ƕ�ת����-PI~0~PI
    if((*x)<0 && (*y)<0)
    {
        Rc_Angle = -PI - Rc_Angle;
    }
    else if((*x)>0 && (*y)<0)
    {
        Rc_Angle = PI - Rc_Angle;
    }
    else if((*x)==0 && (*y)<0)
    {
        Rc_Angle = PI;
    }
    //��ͷ�������
    *x = Rc_Abs*sin(Rc_Angle+angle);
    *y = Rc_Abs*cos(Rc_Angle+angle);
}
/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_RadioContrl_Target(void)
{
    float ftemp=0;

    float roll  = my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.Roll-1500,50);
    float pitch = my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.Pitch-1500,50);
    float yaw   = my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.Yaw-1500,50);

    Rc_Abs = sqrt(roll*roll+ (pitch)*(pitch));

    if(g_FlyControlDataStruct.AppConfigDataStruct.No_Head_Mode)
    {
        Fc_NoHeadMode_Establishment(&roll,&pitch,-g_FlyControlDataStruct.App_AngleStruct.Yaw);
    }

    g_FlyControlDataStruct.AppRCTargetDataStruct.Roll	 = MAX_CTRL_ANGLE*roll*0.002f;
    g_FlyControlDataStruct.AppRCTargetDataStruct.Pitch   = MAX_CTRL_ANGLE*(-pitch)*0.002f;


    if(g_FlyControlDataStruct.AppConfigDataStruct.Thrlow!=1)
    {
        if(g_FlyControlDataStruct.AppConfigDataStruct.LockYaw !=1)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.LockYaw= 1;
            g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw = g_FlyControlDataStruct.App_AngleStruct.Yaw;
        }
    }
    else
    {
        g_FlyControlDataStruct.AppConfigDataStruct.LockYaw = 0;
        g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw   = g_FlyControlDataStruct.App_AngleStruct.Yaw;
    }

    if((my_abs(g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw -g_FlyControlDataStruct.App_AngleStruct.Yaw)<100))
    {
        ftemp=yaw;
        g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw += (ftemp*0.005f)*0.2f;
    }

    //ת[-180.0,+180.0]
    if( g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw >180.0f)  g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw -= 360.0f;
    else if( g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw <-180.0f) g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw += 360.0f;
}
/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_unlock()
{
    if(g_FlyControlDataStruct.AppConfigDataStruct.Startflg==0)  //�Ƿ����
    {
        if(g_FlyControlDataStruct.AppRcDataStruct.THR>1200)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Thrlow=0;//���ŷǵ�
        }
        else
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Thrlow= 1;//��������
        }
    }

    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==0)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.Thrlow= 1;//����״̬�����Ŷ����Ϊ����
    }

    if(g_FlyControlDataStruct.AppConfigDataStruct.NS==3 || g_FlyControlDataStruct.AppConfigDataStruct.NS==0) return;//���źŻ��������źţ����������Ƽ��

    //����
    if(g_FlyControlDataStruct.AppRcDataStruct.THR<1200 && g_FlyControlDataStruct.AppRcDataStruct.Yaw>1750)
    {
        look++;

        if(look>200)
        {
            look = 0;
            g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 0;
        }
    }
    else
    {
        look = 0;
    }

    //�������
    if(g_FlyControlDataStruct.AppRcDataStruct.THR<1200 && g_FlyControlDataStruct.AppRcDataStruct.Yaw<1250)
    {
        unlook++;
        if(unlook>200)
        {
            if(g_FlyControlDataStruct.AppVoltage.V>3.85)//��ѹ�Ͳ��������
            {
                g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 1;
            }
            unlook = 0;
        }
    }
    else
    {
        unlook = 0;
    }

    //У׼���
    if((g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==0) && (g_FlyControlDataStruct.AppRcDataStruct.THR<1200) && (g_FlyControlDataStruct.AppRcDataStruct.Pitch<1200))
    {
        Calibrate_cnt++;

        if(Calibrate_cnt>200)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.calibratingA=1;
            g_FlyControlDataStruct.AppConfigDataStruct.calibratingG=1;
            g_FlyControlDataStruct.AppHightCtrl.BaroHight = 1;
            Calibrate_cnt = 0;
        }
    }
    else
    {
        Calibrate_cnt = 0;
    }
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_fail_safe()
{

    if(g_FlyControlDataStruct.AppConfigDataStruct.NS==0)
    {
        g_FlyControlDataStruct.AppRcDataStruct.Yaw=1500;
        g_FlyControlDataStruct.AppRcDataStruct.Roll=1500;
        g_FlyControlDataStruct.AppRcDataStruct.Pitch=1500;

        if(g_FlyControlDataStruct.AppRcDataStruct.THR>1000)
        {
            g_FlyControlDataStruct.AppRcDataStruct.THR-=1;
        }
        else
        {
            g_FlyControlDataStruct.AppRcDataStruct.THR = 1000;

            if(!g_FlyControlDataStruct.AppConfigDataStruct.FlyMode)
            {
                g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 0;
            }
        }
    }
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_Static_Check(void)
{
    static u8 cnt;

    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==1) return;

    if( my_abs(g_FlyControlDataStruct.AppImuDataStruct.GyroXFilterout) <= 10 &&
            my_abs(g_FlyControlDataStruct.AppImuDataStruct.GyroYFilterout) <= 10 &&
            my_abs(g_FlyControlDataStruct.AppImuDataStruct.GyroZFilterout) <= 10
      )		cnt ++;
    else 	cnt = 0;

    //��ֹ5s��߶�����
    if(cnt>=100) g_FlyControlDataStruct.AppHightCtrl.baroStart = 1;
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/

void Fc_Flag_Check(void)
{
    test_flag = 0;

    //if(!MPU_Err)		test_flag |= BIT0;
    //if(!SPL_Err)		test_flag |= BIT1;
    //if(!NRF_Err)		test_flag |= BIT2;
    //if(ult_ok)    	test_flag |= BIT3;
    //if(!Locat_Err)  test_flag |= BIT4;
    //if(LED_warn==1)	test_flag |= BIT5;//低压
    //if(!Flow_Err)		test_flag |= BIT6;

    set_flag = 0;

    if(g_FlyControlDataStruct.AppConfigDataStruct.FlyMode == 1)	                            set_flag |= BIT0;
    if(g_FlyControlDataStruct.AppConfigDataStruct.Throw == 1)	                            set_flag |= BIT1;
    if(g_FlyControlDataStruct.AppConfigDataStruct.No_Head_Mode == 1)						set_flag |= BIT2;
    if(g_FlyControlDataStruct.AppConfigDataStruct.Fixed_Point_Mode == 1)    				set_flag |= BIT4;

}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void Fc_UnLock(void)
{
    Fc_unlock();
    Fc_fail_safe();//ʧ�ر���
}

void FC_RC_State(void)
{
    NRF_SSI = NRF_SSI_CNT;
    NRF_SSI_CNT = 0;

    Bluet_SSI = Bluet_SSI_CNT;
    Bluet_SSI_CNT = 0;

    if(NRF_SSI || Bluet_SSI)
    {


    }
    else
    {
        g_FlyControlDataStruct.AppConfigDataStruct.NS = 0;
        g_FlyControlDataStruct.AppConfigDataStruct.LedControl= Rc_lost;
    }


}

void Fc_key_function(u8 key)
{
    extern u8 yaw_lock;
    static u8 count1=0;

    if(key&0x02)//��߰���,һ����ɺͽ���
    {
        count1++;
        if(count1>=200)count1=200;
    }
    else
    {
        if(count1>=2)
        {
            //����ģʽ�²�������һ�����
            if(g_FlyControlDataStruct.AppConfigDataStruct.FlyMode ==1)
            {
                if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus ==0)
                {
                    if(g_FlyControlDataStruct.AppVoltage.V>3.8)//��ѹ�Ͳ��������
                    {
                        g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 1;
                        g_FlyControlDataStruct.AppHightCtrl.StartHight = HIGH_START;
                    }
                }
                else
                {
                    g_FlyControlDataStruct.AppConfigDataStruct.LandDown = !g_FlyControlDataStruct.AppConfigDataStruct.LandDown;
                }
            }
        }
        count1=0;
    }
}






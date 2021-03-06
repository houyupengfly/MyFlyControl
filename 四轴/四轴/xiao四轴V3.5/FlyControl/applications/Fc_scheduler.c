/*============================================================================
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

#include "stm32f10x.h"

#include "driver_icm20602.h"
#include "device_mpc2520.h"
#include "driver_Voltage.h"
#include "device_flow.h"
#include "Fc_data_Struct.h"
#include "Fc_scheduler.h"
#include "Fc_systream_time.h"
#include "Fc_SeedEstimated.h"
#include "Fc_revice.h"
#include "Fc_data_transfer.h"
#include "Fc_control.h"
#include "Fc_Parameter.h"

#include "Algorithm_ahrs.h"
#include "stdio.h"

/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/* 定义飞控总数据结构 */
FlyControlDataStruct g_FlyControlDataStruct;

/*============================ STATIC VARIABLES ==============================*/
u16 i,ms;
u32 time[10],time_sum; //系统总运行时间计算方便优化
/*============================ FUNCTION ======================================*/

/*----------------------------------------------------------
 + 实现功能：2ms周期任务
----------------------------------------------------------*/
void Fc_Task_2ms()
{
    time[0] = GetSysTime_us();
    /* mpu6轴传感器数据处理 */
    Call_ICM20602_Data_Prepare();
    Fc_All_Control();
   
    time[0] = GetSysTime_us() - time[0];
}

/*----------------------------------------------------------
 + 实现功能：4ms周期任务
----------------------------------------------------------*/
void Fc_Task_4ms()
{
    time[1] = GetSysTime_us();
    /* 遥控器通道数据处理 */
    Fc_RadioContrl();

    /*遥控输入目标姿态角度计算*/
    Fc_RadioContrl_Target();

    time[1] = GetSysTime_us() - time[1];
}

/*----------------------------------------------------------
 + 实现功能：6ms周期任务
----------------------------------------------------------*/
void Fc_Task_6ms()
{
    time[2] = GetSysTime_us();

    /* IMU更新姿态:ROL,PIT,YAW姿态角 */

    Fc_AHRS_Geteuler(0.006f);
    Fc_Mixedcontrol();

    time[2] = GetSysTime_us() - time[2];

}
/*----------------------------------------------------------
 + 实现功能：10ms周期任务
----------------------------------------------------------*/
void Fc_Task_10ms()
{
    time[3] = GetSysTime_us();
    Fc_UnLock();
    /* 气压计气压与温度数据获取 */
    Fc_Hight_Get(0.01f);
    Fc_Alti_Estimate(10);
    Flow_High_Cal();
    Fc_Hight_Control();
    ReadMotion();
    Fc_Position_Estimate(20);
    Fc_Data_transfer();
    time[3] = GetSysTime_us() - time[3];
}

/*----------------------------------------------------------
 + 实现功能：20ms周期任务
----------------------------------------------------------*/
void Fc_Task_20ms()
{
    time[4] = GetSysTime_us();
    Fc_Flag_Check();
    time[4] = GetSysTime_us() - time[4];
}

/*----------------------------------------------------------
 + 实现功能：50ms周期任务
----------------------------------------------------------*/
void Fc_Task_50ms()
{
    time[5] = GetSysTime_us();
    Fc_Static_Check();
    Fc_voltage_check();
    Fc_PID_Save_Overtime(1500,50);
    time[5] = GetSysTime_us() - time[5];
}

/*----------------------------------------------------------
 + 实现功能：100ms周期任务
----------------------------------------------------------*/
void Fc_Task_100ms()
{
    time[6] = GetSysTime_us();

    time[6] = GetSysTime_us() - time[6];
}

/*----------------------------------------------------------
 + 实现功能：1000ms周期任务
----------------------------------------------------------*/
void Fc_Task_1000ms()
{
    FC_RC_State();
    time_sum = 0;
    for(u8 i=0; i<6; i++)	time_sum += time[i];
}

/*----------------------------------------------------------
 + 实现功能：主循环 由主函数调用
----------------------------------------------------------*/
void Fc_Task_Loop()
{
    //循环周期为2ms
    if(g_FlyControlDataStruct.AppConfigDataStruct.check_flag >= 1)
    {
        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_2ms >=1)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_2ms =0;
            Fc_Task_2ms();     //周期2ms的任务
        }
        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_4ms >= 2)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_4ms = 0;
            Fc_Task_4ms();     //周期4ms的任务
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_6ms >= 3)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_6ms = 0;
            Fc_Task_6ms();     //周期6ms的任务
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_10ms >= 5)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_10ms = 0;
            Fc_Task_10ms();    //周期10ms的任务
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_20ms >= 10)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_20ms = 0;
            Fc_Task_20ms();    //周期20ms的任务
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_50ms >= 25)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_50ms = 0;
            Fc_Task_50ms();    //周期50ms的任务
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_100ms >= 50)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_100ms = 0;
            Fc_Task_100ms();
        }

        if(g_FlyControlDataStruct.AppConfigDataStruct.loop_1000ms >= 500)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.loop_1000ms = 0;
            Fc_Task_1000ms();
            ms++;
        }


    }

    /* 循环运行完毕 标志清零 */
    g_FlyControlDataStruct.AppConfigDataStruct.check_flag = 0;
}

/*----------------------------------------------------------
 + 实现功能：由Systick定时器调用 周期：2毫秒
----------------------------------------------------------*/
void Fc_Call_Loop_timer()
{
    static u8 cnt;

    cnt++;

    /* 不同周期的执行任务独立计时 */
    g_FlyControlDataStruct.AppConfigDataStruct.loop_2ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_4ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_6ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_10ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_20ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_50ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_100ms++;
    g_FlyControlDataStruct.AppConfigDataStruct.loop_1000ms++;

    /* 如果代码在预定周期内没有运行完 */
    if(g_FlyControlDataStruct.AppConfigDataStruct.check_flag >= 1)
        /* 错误次数计数 */
        g_FlyControlDataStruct.AppConfigDataStruct.err_flag ++;
    /* 循环运行开始 标志置1 */
    else
        g_FlyControlDataStruct.AppConfigDataStruct.check_flag += 1;

    /* 等待数传数据开始发送 */
    static short time_1ms;

    /* 上电延时 标志置1 */
    if(time_1ms == 1024)
    {
        /* 上电延时自动校准气压计 */
//        g_FlyControlDataStruct.AppHightCtrl.baroStart = 1;
        /* 上电延时后发送数据 */
        wait_for_translate = 1;
        time_1ms++;
    }
    else if(time_1ms < 1024)
        time_1ms++;
}



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
============================== INCLUDES ======================================*/
#include "Fc_init.h"
#include "Fc_data_Struct.h"
#include "Fc_data_transfer.h"
#include "Fc_Parameter.h"
#include "Fc_systream_time.h"
#include "Fc_control.h"

#include "device_timer.h"
#include "device_pwm_out.h"
#include "device_iic.h"
#include "device_spi.h"
#include "device_icm20602.h"
#include "device_mpc2520.h"
#include "device_Nrf24l01.h"
#include "device_Bluetooth.h"
#include "device_flow.h"
#include "device_ADC.h"
#include "device_led.h"

/*============================ INCLUDES ======================================*/


/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
extern ParamStatus_EnumType g_ParamStatusStruct;
extern u8 hard_error_mpc2520;
u8 hard_error_ICM20602;
/*============================ STATIC VARIABLES ==============================*/
u8 init_finish;
u8 nrf_eanble;
/*============================ FUNCTION ======================================*/
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fly_Control_Init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    //初始化系统滴答定时器
    cycleCounterInit();
    SysTick_Config(SystemCoreClock / 1000);
    /* 初始化信号输出功能24kHZ*/
    Fc_Pwm_Out_Config();

    /*SPI 初始化*/
    Fc_Spi_Init();
    Delay_ms(200);
    Fc_spi2_Init();
    /*NRF24L01初始化*/
    nrf_eanble = Nrf_Check();
    Delay_ms(200);
    //如果连接正常，则将NRF初始化
    if(nrf_eanble)
    {
        Fc_Nrf_Init(MODEL_RX2,80);
    }
    /* 参数初始化 */
    Fc_Param_Read();
    /* I2C初始化 */
    Fc_I2c_Init();
    Delay_ms(200);
    /* 气压计初始化 */
    Fc_Spl0601_Init();
    Delay_ms(200);

    /* 加速度计、陀螺仪初始化，配置低通 */
    Fc_TIM3_OUT();
    if(Drv_Icm20602Reg_Init())
    {
        hard_error_ICM20602=0;
    }
    else
    {
        hard_error_ICM20602=1;
    }
    Delay_ms(200);
    /*蓝牙初始化s*/
    Fc_Blue_Init();
    Delay_ms(200);
    /* 光流模块初始化*/
    PMW3901_Init();
    /* ADC电压检测初始*/
    Fc_Adc_Init();

    /* 硬件故障指示 */
//    if(hard_error_mpc2520)
//    {
//       Fc_led_baro_err();
//    }
//    if(hard_error_ICM20602)
//    {
//       Fc_led_mpu_err();
//    }
    /* 初始化结束标识 */

    g_FlyControlDataStruct.App_AngleStruct.w=1;
    g_FlyControlDataStruct.App_AngleStruct.x=0;
    g_FlyControlDataStruct.App_AngleStruct.y=0;
    g_FlyControlDataStruct.App_AngleStruct.z=0;

    g_FlyControlDataStruct.AppConfigDataStruct.LedControl=Init_OK;
    g_FlyControlDataStruct.AppHightCtrl.baroStart =1;
    init_finish = 1;
}



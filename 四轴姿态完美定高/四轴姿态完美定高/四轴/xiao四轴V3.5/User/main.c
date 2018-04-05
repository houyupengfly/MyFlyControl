/*============================ INCLUDES ======================================
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
#include "Fc_scheduler.h"
/*============================ INCLUDES ======================================*/


/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
int main(void)
{ 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);  //禁用JTAG-DP ，使能 SWD-DP	
	
	Fly_Control_Init();
     
	while(1)
	{
		Fc_Task_Loop();
	}

}







 




















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
#include "Fc_systream_time.h"
#include "Fc_init.h"
#include "Fc_scheduler.h"
/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/

extern u8 init_finish;
/*============================ STATIC VARIABLES ==============================*/
volatile uint32_t usTicks = 0;
// 滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime = 0;

void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

void SysTick_Handler(void)//1ms中断
{
	static u8 cnt;
	
	sysTickUptime++;
	if(!init_finish) return;

	cnt++;	cnt %= 2;
	if(cnt)	Fc_Call_Loop_timer();//2ms执行一次
}

uint32_t GetSysTime_us(void) //两次获取若大于u32/1000(us),则两次差值溢出，不可取
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void Delay_us(__IO u32 nTime)
{ 
	u32 now;
	now = GetSysTime_us();
	while(GetSysTime_us()-now<nTime);
}

void Delay_ms(__IO u32 nTime)
{ 
  u16 i ;
	for(i=0;i<nTime;i++)
	{
		Delay_us(1000);
	}
}

void Delay(vu32 nCount)
{
  for(; nCount!= 0;nCount--);
}



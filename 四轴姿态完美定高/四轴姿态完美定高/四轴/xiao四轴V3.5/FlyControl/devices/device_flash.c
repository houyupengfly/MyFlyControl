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
#include "device_flash.h"
#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 Fc_Flash_Read(u8 *addr, u16 len)	//addr需要写入结构体的地址，len结构体长度
{
    for(u16 i=0; i<len; i=i+2)
    {
        u16 temp;
        if(i+1 <= len-1)
        {
            temp = (*(__IO uint16_t*)(PARAMFLASH_BASE_ADDRESS+i));
            addr[i] = BYTE0(temp);
            addr[i+1] = BYTE1(temp);
        }
        else
        {
            temp = (*(__IO uint16_t*)(PARAMFLASH_BASE_ADDRESS+i));
            addr[i] = BYTE0(temp);
        }

    }
    return 1;
}

u8 Fc_Flash_Write(u8 *addr, u16 len)
{
    uint16_t  FlashStatus;

    //解锁flash
    FLASH_Unlock();

    FlashStatus = FLASH_ErasePage(PARAMFLASH_BASE_ADDRESS);//擦除整页
    if(FlashStatus != FLASH_COMPLETE)
        return 0;

    for(u16 i=0; i<len; i=i+2)
    {
        u16 temp;
        if(i+1 <= len-1)
            temp = (u16)(addr[i+1]<<8) + addr[i];
        else
            temp = 0xff00 + addr[i];

        FlashStatus = FLASH_ProgramHalfWord(PARAMFLASH_BASE_ADDRESS+i, temp);
        if (FlashStatus != FLASH_COMPLETE)
            return 0;
    }
    return 1;
}

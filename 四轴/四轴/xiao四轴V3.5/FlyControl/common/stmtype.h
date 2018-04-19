
/**
* @file     
* @brief    
* @version  
* @author   
* @date     2016-9-26
* @note     
*/

#ifndef STM_TYPE_H
#define STM_TYPE_H

#define STM32_F0 (0)
#define STM32_F1 (1)
#define STM32_F2 (0)
#define STM32_F3 (0)
#define STM32_F4 (0)
#define STM32_F7 (0)

#if (1 != ((STM32_F0) + (STM32_F1) + (STM32_F2) \
  + (STM32_F3) + (STM32_F4) + (STM32_F7)))
#error("stm32fxx config error...")
#endif


#if STM32_F1
#include "stm32f10x.h"
#endif



#endif


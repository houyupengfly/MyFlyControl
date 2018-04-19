/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/
#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

/*============================ INCLUDES ======================================*/

#include "stm32f10x.h"
/*============================ MACROS ========================================*/
#if defined (STM32F10X_LD) || defined (STM32F10X_MD)
#define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#elif defined (STM32F10X_HD) || defined (STM32F10X_CL)
#define PAGE_SIZE  (uint16_t)0x800  /* Page size = 2KByte */
#endif
/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x08010000) /* EEPROM emulation start address: after 64KByte of used Flash memory */
#define PARAMFLASH_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + 0x000))
#define PARAMFLASH_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/

/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
extern u8 Fc_Flash_Write(u8 *addr, u16 len);
extern u8 Fc_Flash_Read(u8 *addr, u16 len);


#endif

#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <stm32f10x.h>


extern u8 Bluet_Connect(void);
extern void Bluet_Receive_Prepare(u8 data);
extern void Fc_Blue_Init(void);

#endif

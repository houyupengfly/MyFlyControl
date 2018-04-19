/**
* @file     App_rc.h
* @brief
* @version  V1.0
* @author   HYP
* @date     2017-4-27
* @note
*/
#ifndef   _APP_RC_H__
#define   _APP_RC_H__

/*============================ INCLUDES ======================================*/

#include "basetype.h"
#include "Fc_data_Struct.h"

/*============================ MACROS =========================================*/



/*============================ TYPES =========================================*/


typedef struct SlaveRCDatasendMaster
{
    s16 THR;
    s16 Yaw;
    s16 Roll;
    s16 Pitch;
    s16 AUX1;
    s16 AUX2;
    s16 AUX3;
    s16 AUX4;
    s16 AUX5;
    s16 AUX6;
    s16 mode;

} SlaveRCDatasendMasterStruct;
/*============================ GLOBAL VARIABLES ===============================*/



/*============================ PROTOTYPES ====================================*/
extern void Fc_RadioContrl(void);
extern void Fc_RadioContrl_Target(void);
extern void Fc_UnLock(void);
extern void Fc_key_function(u8 key);
extern void Fc_Static_Check(void);
extern void Fc_Flag_Check(void);
extern void FC_RC_State(void);
#endif


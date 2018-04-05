/**
* @file
* @brief
* @version
* @author
* @date     2017-2-9
* @note
*/
#ifndef  _DRV_CONTROL_H__
#define  _DRV_CONTROL_H__

/*============================ INCLUDES ======================================*/

#include "basetype.h"
#include "Fc_data_Struct.h"
/*============================ MACROS =========================================*/


/*============================ TYPES =========================================*/


typedef enum MultiType {
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6P = 7,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_FREEMIX = 18,
    MULTITYPE_FLYING_WING = 21
} MultiType;

/*============================ GLOBAL VARIABLES ===============================*/

#define Airfoil MULTITYPE_QUADP

/*============================ PROTOTYPES ====================================*/

extern void  Fc_Hight_Control(void);
extern void  Fc_Mixedcontrol(void);
extern void  Fc_All_Control(void);
extern void  LangFlag(void);
#endif


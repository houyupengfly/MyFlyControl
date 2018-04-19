
#ifndef __APP_PARAMETER_H
#define __APP_PARAMETER_H
#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------- INCLUDES --------------------------------------*/
#include "basetype.h"


/*---------------------------- MACROS ----------------------------------------*/
/*---------------------------- MACROFIED FUNCTIONS ---------------------------*/
/*---------------------------- TYPES -----------------------------------------*/

//! 12 byte
typedef struct {
    float x;
    float y;
    float z;
} ParamIMU_Type;

typedef struct {
    s16 x;
    s16 y;
    s16 z;
} ParamMag_Type;


typedef struct {
    float Kp;
    float Ki;
    float Kd;
} ParamPID_Type;





typedef struct
{
    u8 ParamStatusFlag;

    //! Imu部分
    ParamIMU_Type IMU1_Up_AccBias;
    ParamIMU_Type IMU1_Down_AccBias;
    ParamIMU_Type IMU1_Left_AccBias;
    ParamIMU_Type IMU1_Right_AccBias;
    ParamIMU_Type IMU1_NoseUp_AccBias;
    ParamIMU_Type IMU1_NoseDown_AccBias;

    ParamIMU_Type IMU1_GyroBias;


    //! 磁力计
    ParamMag_Type calMagnetMax;
    ParamMag_Type calMagnetMin;
    ParamMag_Type calMagnetOffset;	//! 磁力计偏移误差
    ParamMag_Type calMagnetS;		//! 校准系数


    //! PID 参数
    //! 姿态控制
    ParamPID_Type Pitch_In_Loop;
    ParamPID_Type Roll_In_Loop;
    ParamPID_Type Yaw_In_Loop;

    ParamPID_Type Pitch_Out_Loop;
    ParamPID_Type Roll_Out_Loop;
    ParamPID_Type Yaw_Out_Loop;

    //! 高度控制
    ParamPID_Type Alti_In_Loop;
    ParamPID_Type Alti_Out_Loop;

    //! 位置控制
    ParamPID_Type X_speed;
    ParamPID_Type Y_speed;

    ParamPID_Type X_positon;
    ParamPID_Type Y_positon;

    float BodyPitchDec;
    float BodyRollDec;



} ParamCtl_Type;




typedef enum {
    Normal = 0,
    WriteAllParam,
    ReadAllParam,
    Err,
} ParamStatus_EnumType;


/*---------------------------- GLOBAL VARIABLES ------------------------------*/
/*---------------------------- LOCAL VARIABLES -------------------------------*/
/*---------------------------- PROTOTYPES ------------------------------------*/
extern void Fc_Param_Read(void);
extern void Fc_PID_Save_Overtime(u16 ms,u16 dTms);
extern void Fc_Para_Init(void);
extern void Param_Save(void);
#ifdef __cplusplus
}
#endif
#endif





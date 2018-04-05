/**
* @file     FlycontrolDataStruct.h
* @brief    软件系统总结构体
* @version  V1.0
* @author   HYP
* @date     2017-4-26
* @note
*/
#ifndef  _FLYCONTROL_DATASTRUCT_H__
#define  _FLYCONTROL_DATASTRUCT_H__

/*============================ INCLUDES ======================================*/

#include "basetype.h"

/*============================ MACROS =========================================*/
#define MAX_CTRL_ANGLE			10.0f

/*============================ TYPES =========================================*/

#define MANUAL_High      1
#define ULTRASONIC_High  2
#define ATMOSPHERE_High  3

#define YAW      0
#define PITCH    1
#define ROLL     2

#define BIT0	0x0001
#define BIT1	0x0002
#define BIT2	0x0004
#define BIT3	0x0008
#define BIT4	0x0010
#define BIT5	0x0020
#define BIT6	0x0040
#define BIT7	0x0080
#define BIT8	0x0100
#define BIT9	0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT15   0x8000

#define HIGH_START  80
#define ANGLETORAD  57.3f
#define PI 3.14f
/*============================ GLOBAL VARIABLES ===============================*/
/* 姿态传感器数据结构 */

typedef struct  AppImuData
{
    s16  iAccX;
    s16  iAccY;
    s16  iAccZ;
    s16  iGyroX;
    s16  iGyroY;
    s16  iGyroZ;

    float AccX;
    float AccY;
    float AccZ;
    float GyroX;
    float GyroY;
    float GyroZ;

    float AccXFilterout;
    float AccYFilterout;
    float AccZFilterout;

    float AccXFilteroutcm;
    float AccYFilteroutcm;
    float AccZFilteroutcm;

    float GyroXFilterout;
    float GyroYFilterout;
    float GyroZFilterout;

    float GyroXFilteroutdeg;
    float GyroYFilteroutdeg;
    float GyroZFilteroutdeg;

    float HAccXFilterout;
    float HAccYFilterout;
    float HAccZFilterout;

    float HGyroXFilterout;
    float HGyroYFilterout;
    float HGyroZFilterout;

    float AccXoffest;
    float AccYoffest;
    float AccZoffest;

    float GyroXoffest;
    float GyroYoffest;
    float GyroZoffest;

    float Nx;
    float Ny;
    float Nz;

    float Bx;
    float By;
    float Bz;

    float mag[3];
} App_ImuDataStruct;

/* 气压传感器数据结构 */
typedef struct AppBaroData
{
    float Pruess;
    float Temp;

} App_BaroDataStruct;

/* 遥控接受数据结构 */
typedef struct AppRcData
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

} App_RCDataStruct;

/*遥控输入目标数据结构  */
typedef struct Target
{
    float Pitch;
    float Roll;
    float Yaw;
    float Altiude;

} App_RCTargetStruct;

/*飞控状态标识控制结构体  */
typedef struct config
{
    u8 AirStatus;
    u8 loop_1ms;
    u8 loop_2ms;
    u8 loop_4ms;
    u8 loop_6ms;
    u8 loop_10ms;
    u8 loop_20ms;
    u8 loop_50ms;
    u8 loop_100ms;
    u16 loop_1000ms;
    u8 err_flag;
    u8 check_flag;
    u8 Startflg;
    u8 MpuExist;
    u8 NrfExist;      // NRF存在
    u8 Flowflag;      // 光流可用
    u8 BaroExist;
    u8 Thrlow;
    u8 NS;            // 信号来源
    u8 LockYaw;       // 航向锁定
    u8 calibratingR;  // 遥控器校准
    u8 calibratingA;  // 加速度校准
    u8 calibratingM;  // 磁力计校准
    u8 calibratingOK; //磁力计预采集
    u8 calibratingG;
    u8 ParamSave;     // 参数保存标志
    u8 PIDReaad;
    u8 FlyMode;
    u8 HightCrtlMode;
    u8 UltFlag;
    u8 BaroFlag;
    u8 LandUp;
    u8 LandDown;
    u8 Throw;
    u8 No_Head_Mode;
    u8 FlowOK;
    u8 LedControl;
    u8 Fixed_Point_Mode;
} App_ConfigDataStruct;
/*算法结构AHRS四元数*/
typedef struct {

    float x;
    float y;
    float z;

} _float_t;
typedef struct Angle
{
    float Pitch;
    float Roll;
    float Yaw;

    float w;//q0;
    float x;//q1;
    float y;//q2;
    float z;//q3;

    _float_t x_vec;
    _float_t y_vec;
    _float_t z_vec;

    _float_t a_acc;
    _float_t w_acc;
    _float_t n_acc;
    _float_t e_acc;

    float RPitch;
    float RRoll;
    float RYaw;
} App_AngleStruct;

typedef struct
{
    float Input_Butter[3];
    float Output_Butter[3];
} Butter_BufferData;

typedef struct
{
    const float a[3];
    const float b[3];
} Butter_Parameter;


typedef struct Pid
{
    float Expect;//期望
    float FeedBack;//反馈值
    float Err;//偏差
    float Last_Err;//上次偏差
    float Err_Max;//偏差限幅值
    float Integrate_Separation_Err;//积分分离偏差值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float p;//控制参数Kp
    float i;//控制参数Ki
    float d;//控制参数Kd
    float Control_OutPut;//控制器总输出
    float Last_Control_OutPut;//上次控制器总输出
    float Control_OutPut_Limit;//输出限幅
    /***************************************/
    float Last_FeedBack;//上次反馈值
    float Dis_Err;//微分量
    float Dis_Error_History[5];//历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲

} PidStruct;

typedef struct PidControl
{
    PidStruct AnglePid;
    PidStruct GyroPid;

} PidControlStruct;

typedef struct Ctrl
{
    s16 Moto[8];
    float ThrControl;
    float HightThrControl;
    PidControlStruct Pitch;
    PidControlStruct Roll;
    PidControlStruct Yaw;
} App_CtrlStruct;

typedef struct HightCtrl
{
    PidStruct HightCtrl;
    PidStruct SpeedCtrl;
    float     BaroHight;
    float     UltHight;
    float     GroundPressure;
    float     GroundHight;
    float     ZHight;
    float     ZSpeed;
    u8        baroStart;
    u8        StartHight;
} App_HightCtrl;

typedef struct PostionCtrl
{
    PidStruct PostionCtrlX;
    PidStruct PostionCtrlY;
    float     HomeX;
    float     HomeY;
} App_PostionCtrl;

typedef struct
{
    float Position[3];//位置估计量
    float Speed[3];//速度估计量
    float Acceleration[3];//加速度估计量
    float Pos_History[3][50];//历史惯导位置
    float Vel_History[3][50];//历史惯导速度
    float Acce_Bias[3];//惯导加速度漂移量估计量
    float Last_Acceleration[3];
    float Last_Speed[3];
    float acc_b[3];		//! 机体坐标系下的acc X,Y,Z
    float acc_n[3];		//! 由机身转换得到的导航acc
    float accNED[3];		//! 在NED下的各轴加速度
} SINS;

typedef struct
{
    float x_offest;
    float y_offest;
    float NSS;
} App_OptflowStruct;

typedef struct
{
    float V;
    float A;
} App_Voltage;
/* 飞控数据总结构体 */
typedef struct Appdata
{
    App_ImuDataStruct      AppImuDataStruct;
    App_BaroDataStruct     AppBaroDataStruct;
    App_RCDataStruct       AppRcDataStruct;
    App_RCTargetStruct     AppRCTargetDataStruct;
    App_ConfigDataStruct   AppConfigDataStruct;
    App_AngleStruct        App_AngleStruct;
    App_CtrlStruct         AppCtrlStruct;
    App_HightCtrl          AppHightCtrl;
    App_PostionCtrl        AppPostionCtrl;
    App_OptflowStruct      AppOptflowStruct;
    App_Voltage            AppVoltage;

} FlyControlDataStruct;


typedef enum {

    Baro_State_Init  = 0,
    Baro_State_Ready,
    Baro_State_Err,

} BaroState_Enum;

typedef enum {

    Ult_State_Init  = 0,
    Ult_State_Ready,
    Ult_State_Err,

} UltState_Enum;

typedef enum {

    Init_OK = 0,
    Bat_low,
    Rc_lost,

} LED_Enum;


/*============================ PROTOTYPES ====================================*/



#endif


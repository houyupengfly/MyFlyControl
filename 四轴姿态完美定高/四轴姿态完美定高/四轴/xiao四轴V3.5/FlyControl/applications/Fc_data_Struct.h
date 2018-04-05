/**
* @file     FlycontrolDataStruct.h
* @brief    ���ϵͳ�ܽṹ��
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
/* ��̬���������ݽṹ */

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

/* ��ѹ���������ݽṹ */
typedef struct AppBaroData
{
    float Pruess;
    float Temp;

} App_BaroDataStruct;

/* ң�ؽ������ݽṹ */
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

/*ң������Ŀ�����ݽṹ  */
typedef struct Target
{
    float Pitch;
    float Roll;
    float Yaw;
    float Altiude;

} App_RCTargetStruct;

/*�ɿ�״̬��ʶ���ƽṹ��  */
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
    u8 NrfExist;      // NRF����
    u8 Flowflag;      // ��������
    u8 BaroExist;
    u8 Thrlow;
    u8 NS;            // �ź���Դ
    u8 LockYaw;       // ��������
    u8 calibratingR;  // ң����У׼
    u8 calibratingA;  // ���ٶ�У׼
    u8 calibratingM;  // ������У׼
    u8 calibratingOK; //������Ԥ�ɼ�
    u8 calibratingG;
    u8 ParamSave;     // ���������־
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
/*�㷨�ṹAHRS��Ԫ��*/
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
    float Expect;//����
    float FeedBack;//����ֵ
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate_Separation_Err;//���ַ���ƫ��ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float p;//���Ʋ���Kp
    float i;//���Ʋ���Ki
    float d;//���Ʋ���Kd
    float Control_OutPut;//�����������
    float Last_Control_OutPut;//�ϴο����������
    float Control_OutPut_Limit;//����޷�
    /***************************************/
    float Last_FeedBack;//�ϴη���ֵ
    float Dis_Err;//΢����
    float Dis_Error_History[5];//��ʷ΢����
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������

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
    float Position[3];//λ�ù�����
    float Speed[3];//�ٶȹ�����
    float Acceleration[3];//���ٶȹ�����
    float Pos_History[3][50];//��ʷ�ߵ�λ��
    float Vel_History[3][50];//��ʷ�ߵ��ٶ�
    float Acce_Bias[3];//�ߵ����ٶ�Ư����������
    float Last_Acceleration[3];
    float Last_Speed[3];
    float acc_b[3];		//! ��������ϵ�µ�acc X,Y,Z
    float acc_n[3];		//! �ɻ���ת���õ��ĵ���acc
    float accNED[3];		//! ��NED�µĸ�����ٶ�
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
/* �ɿ������ܽṹ�� */
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


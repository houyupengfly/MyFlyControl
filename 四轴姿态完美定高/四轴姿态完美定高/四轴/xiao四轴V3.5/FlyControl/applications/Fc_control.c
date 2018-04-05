/**
* @file     Drv_control.c
* @brief    control
* @version  v1.0
* @author   HYP
* @date     2017-2-9
* @note
*/
/*============================ INCLUDES ======================================*/
#include "Fc_data_Struct.h"
#include "Fc_control.h"
#include "Fc_SeedEstimated.h"

#include "Algorithm_math.h"
#include "mymath.h"
#include "string.h"


/*============================ MACROS ========================================*/

#define ThrottleMin           1000
#define ThrottleMax           2000

#define HIGHT_SPEED           100     // cm/s

#define ANGLE_ERR_MAX         30
#define ANGLE_I_MAX           20
#define ANGLE_OUT_MAX         250

#define YAW_ERR_MAX           30
#define YAW_I_MAX             5
#define YAW_OUT_MAX           100

#define GYRO_ERR_I_MAX        200
#define GYRO_ERR_OUT_MAX      500

#define GYRO_ERR_Yaw_MAX      100
#define GYRO_ERR_Yaw_I_MAX    30
#define GYRO_ERR_Yaw_OUT_MAX  120

#define HIGHT_ERR_MAX         150
#define HIGHT_ERR_I_MAX       100
#define HIGHT_ERR_OUT_MAX     400

#define HIGHT_SPEED_ERR_MAX         400
#define HIGHT_SPEED_ERR_I_MAX       500
#define HIGHT_SPEED_ERR_OUT_MAX     700
/*============================ TYPES =========================================*/


/*============================ GLOBAL VARIABLES ==============================*/
extern  FlyControlDataStruct g_FlyControlDataStruct;
extern  SINS NewSinsData;
/*============================ LOCAL VARIABLES ===============================*/
float exp_height_speed;
float exp_height;
float exp_speed;
u16 start_thr=350;
u16 langflag=0;

float err_i_comp;
float ct_val_thr;
u8 AngleConrtolCnt;
u8 ct_alt_hold;
u8 ct_pos_hold;

s16 balance[6],balance_max;
/*============================ IMPLEMENTATION ================================*/
float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    /* 加速度计Butterworth滤波 */
    /* 获取最新x(n) */
    Buffer->Input_Butter[2]=curr_inputer;
    /* Butterworth滤波 */
    Buffer->Output_Butter[2]=
        Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) 序列保存 */
    Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
    Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
    /* y(n) 序列保存 */
    Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
    Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
    return (Buffer->Output_Butter[2]);
}



Butter_Parameter Control_Device_Div_LPF_Parameter= {
//200---20hz
    1,    -1.14298050254,   0.4128015980962,
    0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter= {
    
    //100hz---2hz
    1,   -1.82249692519,   0.83718165125,
    0.00362168151492,    0.0072433630298,  0.0036216815149  
};


float FC_PID_Control_Angal(PidStruct *Controler)
{
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差

    Controler->Err=LIMIT(Controler->Err,-ANGLE_ERR_MAX,ANGLE_ERR_MAX);

    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-ANGLE_I_MAX,ANGLE_I_MAX);

    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                              +Controler->d*(Controler->Err-Controler->Last_Err);//微分
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-ANGLE_OUT_MAX,ANGLE_OUT_MAX);

    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}

float FC_PID_Control_Yaw(PidStruct *Controler)
{
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
    /***********************偏航角偏差超过+-180处理*****************************/
    if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
    if(Controler->Err>180)  Controler->Err=Controler->Err-360;

    Controler->Err=LIMIT(Controler->Err,-YAW_ERR_MAX,YAW_ERR_MAX);
    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-YAW_I_MAX,YAW_I_MAX);
    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                              +Controler->d*(Controler->Err-Controler->Last_Err);//微分
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-YAW_OUT_MAX,YAW_OUT_MAX);
    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}


float FC_Control_Gyro_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
    for(i=4; i>0; i--) //数字低通后微分项保存
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }
    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20h;
    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-GYRO_ERR_I_MAX,GYRO_ERR_I_MAX);
    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                              +Controler->d*Controler->Dis_Err;//微分
                              //+Controler->d*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-GYRO_ERR_OUT_MAX,GYRO_ERR_OUT_MAX);
    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}



float FC_Control_Gyro_Yaw_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
    for(i=4; i>0; i--) //数字低通后微分项保存
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }

    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20h;

    Controler->Err=LIMIT(Controler->Err,-GYRO_ERR_Yaw_MAX,GYRO_ERR_Yaw_MAX);
    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-GYRO_ERR_Yaw_I_MAX,GYRO_ERR_Yaw_I_MAX);
    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                              +Controler->d*Controler->Dis_Err;//微分
                              //+Controler->d*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-GYRO_ERR_Yaw_OUT_MAX,GYRO_ERR_Yaw_OUT_MAX);
    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}

float FC_PID_Control_Hight(PidStruct *Controler)
{
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差

    Controler->Err=LIMIT(Controler->Err,-HIGHT_ERR_MAX,HIGHT_ERR_MAX);

    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-HIGHT_ERR_I_MAX,HIGHT_ERR_I_MAX);

    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                              +Controler->d*(Controler->Err-Controler->Last_Err);//微分
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-HIGHT_ERR_OUT_MAX,HIGHT_ERR_OUT_MAX);

    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}

float FC_Control_HightSpeed_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******偏差计算*********************/
    Controler->Last_Err=Controler->Err;//保存上次偏差
    Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
    
    Controler->Err=LIMIT(Controler->Err,-HIGHT_SPEED_ERR_MAX,HIGHT_SPEED_ERR_MAX);
    
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
    for(i=4; i>0; i--) //数字低通后微分项保存
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }
    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

    /*******积分计算*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******积分限幅*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-HIGHT_SPEED_ERR_I_MAX,HIGHT_SPEED_ERR_I_MAX);
    /*******总输出计算*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
    Controler->Control_OutPut=Controler->p*Controler->Err//比例
                              +Controler->Integrate//积分
                             // +Controler->d*Controler->Dis_Err;//微分
                              +Controler->d*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
    /*******总输出限幅*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-1000+err_i_comp,1000-err_i_comp);
    /*******返回总输出*********************/
    return Controler->Control_OutPut;
}
/********************************************************姿态控制**************************************************************/

void Fc_AttitudeControl(void)
{
    AngleConrtolCnt++;

    if(AngleConrtolCnt==3)
    {
        g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid.Expect   = g_FlyControlDataStruct.AppRCTargetDataStruct.Pitch;

        g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid.FeedBack = g_FlyControlDataStruct.App_AngleStruct.Pitch;

        g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid.Expect    = g_FlyControlDataStruct.AppRCTargetDataStruct.Roll;

        g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid.FeedBack  = g_FlyControlDataStruct.App_AngleStruct.Roll;

        g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid.Expect     = g_FlyControlDataStruct.AppRCTargetDataStruct.Yaw;

        g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid.FeedBack   = g_FlyControlDataStruct.App_AngleStruct.Yaw;


        FC_PID_Control_Angal(&g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid);

        FC_PID_Control_Angal(&g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid);

        FC_PID_Control_Yaw(&g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid);

        AngleConrtolCnt=0;
    }

    g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.Expect=g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid.Control_OutPut;
    g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.FeedBack=-g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg;

    g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Expect=g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid.Control_OutPut;
//    g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Expect=(g_FlyControlDataStruct.AppRcDataStruct.Roll-1500);
    g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.FeedBack=g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg;

    g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.Expect=g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid.Control_OutPut;
    g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.FeedBack=-g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg
            ;

    FC_Control_Gyro_Div_LPF(&g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid);

    FC_Control_Gyro_Div_LPF(&g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid);

    FC_Control_Gyro_Yaw_Div_LPF(&g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid);
}
/********************************************************定高控制**************************************************************/
void AltitudeV(void)
{
    u8 out_en;

    if(g_FlyControlDataStruct.AppConfigDataStruct.Startflg)
    {
        out_en = 1;
    }
    else
    {
        out_en = 0;
    }

    exp_speed= g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut+exp_height_speed;

    g_FlyControlDataStruct.AppHightCtrl.SpeedCtrl.Expect =  exp_speed;
    g_FlyControlDataStruct.AppHightCtrl.SpeedCtrl.FeedBack= NewSinsData.Speed[0];

    FC_Control_HightSpeed_Div_LPF(&g_FlyControlDataStruct.AppHightCtrl.SpeedCtrl);

    if(out_en)
    {
        err_i_comp = start_thr;
    }
    else
    {
        err_i_comp = 0;
        g_FlyControlDataStruct.AppHightCtrl.SpeedCtrl.Integrate=0;
    }

    g_FlyControlDataStruct.AppCtrlStruct.HightThrControl = out_en *(g_FlyControlDataStruct.AppHightCtrl.SpeedCtrl.Control_OutPut + err_i_comp-0.03*g_FlyControlDataStruct.AppImuDataStruct.Nz);
}

void AltitudeH(void)
{
    u8 out_en;

    if(g_FlyControlDataStruct.AppConfigDataStruct.Startflg)
    {
        out_en = 1;
    }
    else
    {
        out_en = 0;
    }

    //遥控高度变化阶段
    if(exp_height_speed != 0)
    {
        ct_alt_hold = 0;//关闭定高
        g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut = 0;//输出为零
    }

    else
    {
         
        //重新回到定高阶段
        if(ct_alt_hold==0)
        {
            //如果反馈速度接近0，启动定高
            if(ABS(NewSinsData.Speed[0])<10)
            {
                ct_alt_hold = 1;
               exp_height =NewSinsData.Position[0]; //记录当前期望高度
            }
            //否则减速
            else
            {
                g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut += (-NewSinsData.Speed[0] - g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut)*0.5f;//减速
            }
        }
    
    }

    if(out_en)
    {
        if(ct_alt_hold == 1)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Throw=0;
            g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Expect = exp_height;
            g_FlyControlDataStruct.AppHightCtrl.HightCtrl.FeedBack=NewSinsData.Position[0];
            FC_PID_Control_Hight(&g_FlyControlDataStruct.AppHightCtrl.HightCtrl);
        }
    }
    else
    {
        exp_height =NewSinsData.Position[0]; //记录当前期望高度
        g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut = 0;
    }

}

void Fc_Hight_Control(void)
{
    static u8 _cnt;

    //标记为未起飞状态
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus == 0 || g_FlyControlDataStruct.AppConfigDataStruct.FlyMode!=1)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 0;
        return;
    }

    if(g_FlyControlDataStruct.AppConfigDataStruct.Startflg  && g_FlyControlDataStruct.AppConfigDataStruct.AirStatus && (g_FlyControlDataStruct.AppCtrlStruct.HightThrControl<100))   //降落判断
    {
        _cnt++;

        if(_cnt>=200)
        {
            _cnt = 0;            //清除标记
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 0;           //标记为未起飞状态
            g_FlyControlDataStruct.AppConfigDataStruct.LandDown = 0;
            g_FlyControlDataStruct.AppConfigDataStruct.AirStatus= 0;			 //上锁
        }
    }
    else _cnt = 0;

    switch(g_FlyControlDataStruct.AppConfigDataStruct.Startflg)
    {
    //没有起飞
    case 0:
        if(g_FlyControlDataStruct.AppRcDataStruct.THR>=1500)
        {
            g_FlyControlDataStruct.AppHightCtrl.baroStart = 1;//高度清零
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 1;//起飞
        }
        else
        {
            exp_height_speed = 0 ;
        }

        break;

    //自动起飞阶段
    case 1:

        //起飞高度大于目标高度
        if(NewSinsData.Position[0] >= g_FlyControlDataStruct.AppHightCtrl.StartHight ||g_FlyControlDataStruct.AppConfigDataStruct.Throw)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 2;
        }
        else
        {
            exp_height_speed = 150 + HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,100)/500.0f;;//自动起飞速度
        }

        exp_height_speed= LIMIT(exp_height_speed,0,200);

        //自动起飞过程中拉低油门，可以取消自动起飞
        if(g_FlyControlDataStruct.AppRcDataStruct.THR< 1100) g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 2;

        break;

    //飞行中
    default:

        //自动降落
        if(g_FlyControlDataStruct.AppConfigDataStruct.LandDown)
        {
            exp_height_speed = -100 + HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,50)/500.0f;;
            exp_height_speed = LIMIT(exp_height_speed,-200,200);
        }
        //遥控控制高度
        else
        {
            exp_height_speed = HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,50)/500.0f;
        }

        break;
    }

    AltitudeV();
    AltitudeH();
}


/******************************************************************光流定点控制*******************************************************************************/
//void Fc_PositionXY(void)
//{
//    u8 out_en;

//    if(g_FlyControlDataStruct.AppConfigDataStruct.Flowflag)
//    {
//        out_en = 1;
//    }
//    else
//    {
//        out_en = 0;
//    }

//    if((my_abs(g_FlyControlDataStruct.AppRCTargetDataStruct.Pitch<1)) && my_abs(g_FlyControlDataStruct.AppRCTargetDataStruct.Roll<1))//无水平遥控量给定右手摇杆期望小于1度
//    {
//        g_FlyControlDataStruct.AppPostionCtrl.HomeX=0;
//        g_FlyControlDataStruct.AppPostionCtrl.HomeY=0;

//位置反馈，来源于当前惯导的位置估计

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err = (g_FlyControlDataStruct.AppPostionCtrl.HomeX - g_FlyControlDataStruct.AppOptflowStruct.x_offest); //PITCH
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err = (g_FlyControlDataStruct.AppPostionCtrl.HomeY - g_FlyControlDataStruct.AppOptflowStruct.y_offest); //ROLL

//        //对位置变化进制限制
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,-20,20);//位置偏差限幅，单位cm
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err,-20,20);//位置偏差限幅，单位cm

//        //对水平X方向进行控制;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold;//原始微分
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF=Control_Device_LPF(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,
//                &g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Control_Device_LPF_Buffer,
//                &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF;//偏差经过低通后的微分量
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold= g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err;//保存上次偏差
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i += g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i*g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i,-10,10);

//        /*******总输出计算*********************/
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout = g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p * g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF//比例
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i//积分
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d *g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通

//        //对水平Y方向进行控制
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold;//原始微分
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF=Control_Device_LPF(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,
//                &g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Control_Device_LPF_Buffer,
//                &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF;//偏差经过低通后的微分量
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold= g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err;//保存上次偏差
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i += g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i*g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i,-10,10);

//        /*******总输出计算*********************/
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout = g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p * g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF//比例
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i//积分
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d *g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout  =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout*out_en,-15,15);
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout  =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout*out_en,-15,15);

//    }
//}

void PositionControl(void)
{
    if(g_FlyControlDataStruct.AppConfigDataStruct.Fixed_Point_Mode == 0)
    {
        return;
    }

    switch(g_FlyControlDataStruct.AppConfigDataStruct.Startflg)
    {
        case 0://没有起飞
            g_FlyControlDataStruct.AppConfigDataStruct.Flowflag = 0;//不定点
            break;

        case 2://自动起飞阶段完成
            g_FlyControlDataStruct.AppConfigDataStruct.Flowflag = 1;		//开启定点
            break;
    }

//    PositionXY();//位置速度环控制
}



void DrvMotoPwm(void)
{
    if(Airfoil ==MULTITYPE_QUADP)
    {
        TIM4->CCR1=0 + g_FlyControlDataStruct.AppCtrlStruct.Moto[0];
        TIM4->CCR2=0 + g_FlyControlDataStruct.AppCtrlStruct.Moto[1];
        TIM4->CCR4=0 + g_FlyControlDataStruct.AppCtrlStruct.Moto[2];
        TIM4->CCR3=0 + g_FlyControlDataStruct.AppCtrlStruct.Moto[3];
    }

}
void DrvMotoStop(void)
{
    if(Airfoil ==MULTITYPE_QUADP)
    {
        TIM4->CCR1=0;
        TIM4->CCR2=0;
        TIM4->CCR3=0;
        TIM4->CCR4=0;
    }
}

s16 Pitch,Roll,Yaw;
/******************************************************************混合动力分配控制*******************************************************************************/
void Fc_Mixedcontrol(void)
{

    Pitch = (s16)g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.Control_OutPut;
    Roll  = (s16)g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Control_OutPut;
    Yaw   = (s16)g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.Control_OutPut;

    if((g_FlyControlDataStruct.AppConfigDataStruct.Thrlow!=1))
    {
        //进入混控判断飞行机型
        switch(Airfoil)
        {
        case MULTITYPE_BI:
            break;

        case MULTITYPE_QUADP:
            /*         CW2     1CCW           */
            /*             \ /                */
            /*             / \                */
            /*        CCW3     4CW            */
            balance[0] =  + Pitch - Roll + Yaw;//1// 右前
            balance[1] =  + Pitch + Roll - Yaw;//2// 左前
            balance[2] =  - Pitch + Roll + Yaw;//3// 左后
            balance[3] =  - Pitch - Roll - Yaw;//4// 右后
            balance_max = 0;

            for(u8 i=0; i<4; i++)
            {
                if(balance_max < ABS(balance[i])) balance_max = balance[i];
            }

            balance_max = LIMIT(balance_max,0,200);

            for(u8 i=0; i<4; i++)
            {
                g_FlyControlDataStruct.AppCtrlStruct.Moto[i] = (s16)g_FlyControlDataStruct.AppCtrlStruct.ThrControl + balance[i] ;
                g_FlyControlDataStruct.AppCtrlStruct.Moto[i] = LIMIT(g_FlyControlDataStruct.AppCtrlStruct.Moto[i],100,1000);
            }

            break;

        case MULTITYPE_HEX6X:
            /*         CW2     1CCW           */
            /*            \   /               */
            /*             \ /                */
            /*   CCW3—— ——     —— ——6CW*/
            /*             / \                */
            /*            /   \               */
            /*         CW4     5CCW           */
            g_FlyControlDataStruct.AppCtrlStruct.Moto[0] = + Pitch - Roll/2 + Yaw;// 1点钟方向，CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[1] = + Pitch + Roll/2 - Yaw;//11点钟方向， CW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[2] =         + Roll   + Yaw;// 9点钟方向，CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[3] = - Pitch + Roll/2 - Yaw;// 7点钟方向， CW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[4] = - Pitch - Roll/2 + Yaw;// 5点钟方向，CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[5] =         - Roll   - Yaw;// 3点钟方向， CW

            for(u8 i=0; i<6; i++)
            {
                g_FlyControlDataStruct.AppCtrlStruct.Moto[i] = (s16)g_FlyControlDataStruct.AppCtrlStruct.ThrControl  + g_FlyControlDataStruct.AppCtrlStruct.Moto[i] ;
                g_FlyControlDataStruct.AppCtrlStruct.Moto[i] = LIMIT(g_FlyControlDataStruct.AppCtrlStruct.Moto[i],0,1000);
            }

            break;
        };
    }
    else
    {
        g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.Integrate=0;
        Pitch = 0;
        Roll  = 0;
        Yaw   = 0;
        g_FlyControlDataStruct.AppCtrlStruct.Moto[0] = 0;
        g_FlyControlDataStruct.AppCtrlStruct.Moto[1] = 0;
        g_FlyControlDataStruct.AppCtrlStruct.Moto[2] = 0;
        g_FlyControlDataStruct.AppCtrlStruct.Moto[3] = 0;
    }

    //如果已经解锁 电机容许输出
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus == 1)
    {
        DrvMotoPwm();
    }
    //没有解锁  电机不可输出
    else if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus == 0)
    {
        g_FlyControlDataStruct.AppCtrlStruct.Pitch.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Roll.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Yaw.AnglePid.Integrate=0;
        g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.Integrate=0;
        DrvMotoStop();
    }
}

/******************************************************************油门定高控制与输出*******************************************************************************/

void Fc_All_Control(void)
{
    float thr,angle;

    thr=g_FlyControlDataStruct.AppRcDataStruct.THR-1000;

    Fc_AttitudeControl();

    if(g_FlyControlDataStruct.AppConfigDataStruct.FlyMode ==1)
    {
        thr=g_FlyControlDataStruct.AppCtrlStruct.HightThrControl;
    }
    //进行油门补偿
    angle = my_sqrt(g_FlyControlDataStruct.App_AngleStruct.Roll*g_FlyControlDataStruct.App_AngleStruct.Roll + g_FlyControlDataStruct.App_AngleStruct.Pitch*g_FlyControlDataStruct.App_AngleStruct.Pitch);
    if(angle>30)	angle=30;	//限幅
    angle /= 57.29f;	 	//角度转弧度
//
    g_FlyControlDataStruct.AppCtrlStruct.ThrControl  = thr;// / my_cos(angle);
    g_FlyControlDataStruct.AppCtrlStruct.ThrControl  = LIMIT(g_FlyControlDataStruct.AppCtrlStruct.ThrControl ,0,1000-balance_max);

}

/******************************************************************特殊功能设置*******************************************************************************/
//抛飞检测
void Throw_Fly_Check(void)
{
    static u8 _cnt;

    //解锁状态下不检测
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==1 || g_FlyControlDataStruct.AppConfigDataStruct.Throw == 0) return;

    //检测上升
    if(_cnt==0 && NewSinsData.Speed[0] > 200)
    {
        _cnt=1;
    }

    //上升到顶点马上启动电机保持平衡
    if(_cnt==1 && NewSinsData.Speed[0] < 50)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 1;
        _cnt = 0;
    }
}


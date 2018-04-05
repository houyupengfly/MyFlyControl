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
    /* ���ٶȼ�Butterworth�˲� */
    /* ��ȡ����x(n) */
    Buffer->Input_Butter[2]=curr_inputer;
    /* Butterworth�˲� */
    Buffer->Output_Butter[2]=
        Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) ���б��� */
    Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
    Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
    /* y(n) ���б��� */
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
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��

    Controler->Err=LIMIT(Controler->Err,-ANGLE_ERR_MAX,ANGLE_ERR_MAX);

    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-ANGLE_I_MAX,ANGLE_I_MAX);

    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                              +Controler->d*(Controler->Err-Controler->Last_Err);//΢��
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-ANGLE_OUT_MAX,ANGLE_OUT_MAX);

    /*******���������*********************/
    return Controler->Control_OutPut;
}

float FC_PID_Control_Yaw(PidStruct *Controler)
{
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
    /***********************ƫ����ƫ���+-180����*****************************/
    if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
    if(Controler->Err>180)  Controler->Err=Controler->Err-360;

    Controler->Err=LIMIT(Controler->Err,-YAW_ERR_MAX,YAW_ERR_MAX);
    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-YAW_I_MAX,YAW_I_MAX);
    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                              +Controler->d*(Controler->Err-Controler->Last_Err);//΢��
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-YAW_OUT_MAX,YAW_OUT_MAX);
    /*******���������*********************/
    return Controler->Control_OutPut;
}


float FC_Control_Gyro_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
    for(i=4; i>0; i--) //���ֵ�ͨ��΢�����
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }
    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20h;
    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-GYRO_ERR_I_MAX,GYRO_ERR_I_MAX);
    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                              +Controler->d*Controler->Dis_Err;//΢��
                              //+Controler->d*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-GYRO_ERR_OUT_MAX,GYRO_ERR_OUT_MAX);
    /*******���������*********************/
    return Controler->Control_OutPut;
}



float FC_Control_Gyro_Yaw_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
    for(i=4; i>0; i--) //���ֵ�ͨ��΢�����
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }

    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20h;

    Controler->Err=LIMIT(Controler->Err,-GYRO_ERR_Yaw_MAX,GYRO_ERR_Yaw_MAX);
    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-GYRO_ERR_Yaw_I_MAX,GYRO_ERR_Yaw_I_MAX);
    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                              +Controler->d*Controler->Dis_Err;//΢��
                              //+Controler->d*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-GYRO_ERR_Yaw_OUT_MAX,GYRO_ERR_Yaw_OUT_MAX);
    /*******���������*********************/
    return Controler->Control_OutPut;
}

float FC_PID_Control_Hight(PidStruct *Controler)
{
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��

    Controler->Err=LIMIT(Controler->Err,-HIGHT_ERR_MAX,HIGHT_ERR_MAX);

    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-HIGHT_ERR_I_MAX,HIGHT_ERR_I_MAX);

    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                              +Controler->d*(Controler->Err-Controler->Last_Err);//΢��
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-HIGHT_ERR_OUT_MAX,HIGHT_ERR_OUT_MAX);

    /*******���������*********************/
    return Controler->Control_OutPut;
}

float FC_Control_HightSpeed_Div_LPF(PidStruct *Controler)
{
    s16  i=0;
    /*******ƫ�����*********************/
    Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
    Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
    
    Controler->Err=LIMIT(Controler->Err,-HIGHT_SPEED_ERR_MAX,HIGHT_SPEED_ERR_MAX);
    
    Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
    for(i=4; i>0; i--) //���ֵ�ͨ��΢�����
    {
        Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
    }
    Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                    &Controler->Control_Device_LPF_Buffer,
                                    &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

    /*******���ּ���*********************/
    Controler->Integrate+=Controler->i*Controler->Err;

    /*******�����޷�*********************/
    Controler->Integrate=LIMIT(Controler->Integrate,-HIGHT_SPEED_ERR_I_MAX,HIGHT_SPEED_ERR_I_MAX);
    /*******���������*********************/
    Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
    Controler->Control_OutPut=Controler->p*Controler->Err//����
                              +Controler->Integrate//����
                             // +Controler->d*Controler->Dis_Err;//΢��
                              +Controler->d*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
    /*******������޷�*********************/
    Controler->Control_OutPut=LIMIT(Controler->Control_OutPut,-1000+err_i_comp,1000-err_i_comp);
    /*******���������*********************/
    return Controler->Control_OutPut;
}
/********************************************************��̬����**************************************************************/

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
/********************************************************���߿���**************************************************************/
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

    //ң�ظ߶ȱ仯�׶�
    if(exp_height_speed != 0)
    {
        ct_alt_hold = 0;//�رն���
        g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut = 0;//���Ϊ��
    }

    else
    {
         
        //���»ص����߽׶�
        if(ct_alt_hold==0)
        {
            //��������ٶȽӽ�0����������
            if(ABS(NewSinsData.Speed[0])<10)
            {
                ct_alt_hold = 1;
               exp_height =NewSinsData.Position[0]; //��¼��ǰ�����߶�
            }
            //�������
            else
            {
                g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut += (-NewSinsData.Speed[0] - g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut)*0.5f;//����
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
        exp_height =NewSinsData.Position[0]; //��¼��ǰ�����߶�
        g_FlyControlDataStruct.AppHightCtrl.HightCtrl.Control_OutPut = 0;
    }

}

void Fc_Hight_Control(void)
{
    static u8 _cnt;

    //���Ϊδ���״̬
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus == 0 || g_FlyControlDataStruct.AppConfigDataStruct.FlyMode!=1)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 0;
        return;
    }

    if(g_FlyControlDataStruct.AppConfigDataStruct.Startflg  && g_FlyControlDataStruct.AppConfigDataStruct.AirStatus && (g_FlyControlDataStruct.AppCtrlStruct.HightThrControl<100))   //�����ж�
    {
        _cnt++;

        if(_cnt>=200)
        {
            _cnt = 0;            //������
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 0;           //���Ϊδ���״̬
            g_FlyControlDataStruct.AppConfigDataStruct.LandDown = 0;
            g_FlyControlDataStruct.AppConfigDataStruct.AirStatus= 0;			 //����
        }
    }
    else _cnt = 0;

    switch(g_FlyControlDataStruct.AppConfigDataStruct.Startflg)
    {
    //û�����
    case 0:
        if(g_FlyControlDataStruct.AppRcDataStruct.THR>=1500)
        {
            g_FlyControlDataStruct.AppHightCtrl.baroStart = 1;//�߶�����
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 1;//���
        }
        else
        {
            exp_height_speed = 0 ;
        }

        break;

    //�Զ���ɽ׶�
    case 1:

        //��ɸ߶ȴ���Ŀ��߶�
        if(NewSinsData.Position[0] >= g_FlyControlDataStruct.AppHightCtrl.StartHight ||g_FlyControlDataStruct.AppConfigDataStruct.Throw)
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 2;
        }
        else
        {
            exp_height_speed = 150 + HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,100)/500.0f;;//�Զ�����ٶ�
        }

        exp_height_speed= LIMIT(exp_height_speed,0,200);

        //�Զ���ɹ������������ţ�����ȡ���Զ����
        if(g_FlyControlDataStruct.AppRcDataStruct.THR< 1100) g_FlyControlDataStruct.AppConfigDataStruct.Startflg = 2;

        break;

    //������
    default:

        //�Զ�����
        if(g_FlyControlDataStruct.AppConfigDataStruct.LandDown)
        {
            exp_height_speed = -100 + HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,50)/500.0f;;
            exp_height_speed = LIMIT(exp_height_speed,-200,200);
        }
        //ң�ؿ��Ƹ߶�
        else
        {
            exp_height_speed = HIGHT_SPEED *my_deathzoom_2(g_FlyControlDataStruct.AppRcDataStruct.THR - 1500,50)/500.0f;
        }

        break;
    }

    AltitudeV();
    AltitudeH();
}


/******************************************************************�����������*******************************************************************************/
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

//    if((my_abs(g_FlyControlDataStruct.AppRCTargetDataStruct.Pitch<1)) && my_abs(g_FlyControlDataStruct.AppRCTargetDataStruct.Roll<1))//��ˮƽң������������ҡ������С��1��
//    {
//        g_FlyControlDataStruct.AppPostionCtrl.HomeX=0;
//        g_FlyControlDataStruct.AppPostionCtrl.HomeY=0;

//λ�÷�������Դ�ڵ�ǰ�ߵ���λ�ù���

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err = (g_FlyControlDataStruct.AppPostionCtrl.HomeX - g_FlyControlDataStruct.AppOptflowStruct.x_offest); //PITCH
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err = (g_FlyControlDataStruct.AppPostionCtrl.HomeY - g_FlyControlDataStruct.AppOptflowStruct.y_offest); //ROLL

//        //��λ�ñ仯��������
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,-20,20);//λ��ƫ���޷�����λcm
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.err,-20,20);//λ��ƫ���޷�����λcm

//        //��ˮƽX������п���;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold;//ԭʼ΢��
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF=Control_Device_LPF(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,
//                &g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Control_Device_LPF_Buffer,
//                &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF;//ƫ�����ͨ���΢����
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold= g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err;//�����ϴ�ƫ��
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i += g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i*g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i,-10,10);

//        /*******���������*********************/
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout = g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p * g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF//����
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i//����
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d *g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ

//        //��ˮƽY������п���
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold;//ԭʼ΢��
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF=Control_Device_LPF(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err,
//                &g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Control_Device_LPF_Buffer,
//                &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF-g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF;//ƫ�����ͨ���΢����
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Last_Err_LPF=g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.errold= g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.err;//�����ϴ�ƫ��
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i += g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i*g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF;
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i =LIMIT(g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i,-10,10);

//        /*******���������*********************/
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.pidout = g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p * g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Err_LPF//����
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i//����
//                +g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d *g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ
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
        case 0://û�����
            g_FlyControlDataStruct.AppConfigDataStruct.Flowflag = 0;//������
            break;

        case 2://�Զ���ɽ׶����
            g_FlyControlDataStruct.AppConfigDataStruct.Flowflag = 1;		//��������
            break;
    }

//    PositionXY();//λ���ٶȻ�����
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
/******************************************************************��϶����������*******************************************************************************/
void Fc_Mixedcontrol(void)
{

    Pitch = (s16)g_FlyControlDataStruct.AppCtrlStruct.Pitch.GyroPid.Control_OutPut;
    Roll  = (s16)g_FlyControlDataStruct.AppCtrlStruct.Roll.GyroPid.Control_OutPut;
    Yaw   = (s16)g_FlyControlDataStruct.AppCtrlStruct.Yaw.GyroPid.Control_OutPut;

    if((g_FlyControlDataStruct.AppConfigDataStruct.Thrlow!=1))
    {
        //�������жϷ��л���
        switch(Airfoil)
        {
        case MULTITYPE_BI:
            break;

        case MULTITYPE_QUADP:
            /*         CW2     1CCW           */
            /*             \ /                */
            /*             / \                */
            /*        CCW3     4CW            */
            balance[0] =  + Pitch - Roll + Yaw;//1// ��ǰ
            balance[1] =  + Pitch + Roll - Yaw;//2// ��ǰ
            balance[2] =  - Pitch + Roll + Yaw;//3// ���
            balance[3] =  - Pitch - Roll - Yaw;//4// �Һ�
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
            /*   CCW3���� ����     ���� ����6CW*/
            /*             / \                */
            /*            /   \               */
            /*         CW4     5CCW           */
            g_FlyControlDataStruct.AppCtrlStruct.Moto[0] = + Pitch - Roll/2 + Yaw;// 1���ӷ���CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[1] = + Pitch + Roll/2 - Yaw;//11���ӷ��� CW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[2] =         + Roll   + Yaw;// 9���ӷ���CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[3] = - Pitch + Roll/2 - Yaw;// 7���ӷ��� CW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[4] = - Pitch - Roll/2 + Yaw;// 5���ӷ���CCW
            g_FlyControlDataStruct.AppCtrlStruct.Moto[5] =         - Roll   - Yaw;// 3���ӷ��� CW

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

    //����Ѿ����� ����������
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus == 1)
    {
        DrvMotoPwm();
    }
    //û�н���  ����������
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

/******************************************************************���Ŷ��߿��������*******************************************************************************/

void Fc_All_Control(void)
{
    float thr,angle;

    thr=g_FlyControlDataStruct.AppRcDataStruct.THR-1000;

    Fc_AttitudeControl();

    if(g_FlyControlDataStruct.AppConfigDataStruct.FlyMode ==1)
    {
        thr=g_FlyControlDataStruct.AppCtrlStruct.HightThrControl;
    }
    //�������Ų���
    angle = my_sqrt(g_FlyControlDataStruct.App_AngleStruct.Roll*g_FlyControlDataStruct.App_AngleStruct.Roll + g_FlyControlDataStruct.App_AngleStruct.Pitch*g_FlyControlDataStruct.App_AngleStruct.Pitch);
    if(angle>30)	angle=30;	//�޷�
    angle /= 57.29f;	 	//�Ƕ�ת����
//
    g_FlyControlDataStruct.AppCtrlStruct.ThrControl  = thr;// / my_cos(angle);
    g_FlyControlDataStruct.AppCtrlStruct.ThrControl  = LIMIT(g_FlyControlDataStruct.AppCtrlStruct.ThrControl ,0,1000-balance_max);

}

/******************************************************************���⹦������*******************************************************************************/
//�׷ɼ��
void Throw_Fly_Check(void)
{
    static u8 _cnt;

    //����״̬�²����
    if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==1 || g_FlyControlDataStruct.AppConfigDataStruct.Throw == 0) return;

    //�������
    if(_cnt==0 && NewSinsData.Speed[0] > 200)
    {
        _cnt=1;
    }

    //�������������������������ƽ��
    if(_cnt==1 && NewSinsData.Speed[0] < 50)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.AirStatus = 1;
        _cnt = 0;
    }
}


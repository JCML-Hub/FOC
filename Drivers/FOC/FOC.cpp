#include "FOC.hpp"
#include "tim.h"
#include "adc.h"
#include "dma.h"
#include "DTek_TLE5012B.h"
#include "function.h"
#include "utilities.h"
#include "Time_utils.h"

#define _3PI_2  4.71238898f

unsigned long timestamp;
float shaft_angle, Target=-25, TargetSpeed = 10.0f, TargetAngle = 1.0f;
float Last_Angle=0;

fp32 Uq_PID_Param[5] = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f};
fp32 Speed_PID_Param[5] = {0.1f, 0.0f, 0.0f, MODULATION/2, MODULATION/2};
fp32 Angle_PID_Param[5] = {0.3f, 0.0f, 0.0f, 10.0f, 10.0f};

inline float abs(float _v)
{
  return _v >= 0 ? _v : -_v;
}

void FOC::Init(){
  Theta = 0;
  Target_Udq.x1 = 0.0f; //  D value
  Target_Udq.x2 = 0.2f;  //  Q Value
  // 底层初始化
  TEL5012B_Init();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_ADCEx_Calibration_Start(&hadc1);//校准ADC
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Value,4);
//  hdma_adc1.XferCpltCallback = ADC_DMACallback;
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);

  FOC_electrical_angle_calibration();
  //算法初始化f
  LowPassFilter_Init(&Iq_filter, 0.004f);
  LowPassFilter_Init(&Id_filter, 0.004f);
  LowPassFilter_Init(&speed, 0.1f);

  pid_init(&Iq_PID, PID_POSITION, Uq_PID_Param);
  pid_init(&Id_PID, PID_POSITION, Uq_PID_Param);
  pid_init(&Speed_PID, PID_POSITION, Speed_PID_Param);
  pid_init(&Angle_PID, PID_POSITION, Angle_PID_Param);
}

void FOC::Update(){
  /*输入解算*/
  Encoder_Update();
  Iabc.x1 = (float )(3813 - ADC_Value[0]) * Current_Gain;
  Iabc.x2 = (float )(3887 - ADC_Value[1]) * Current_Gain;
  Iabc.x3 = (float )(3875 - ADC_Value[2]) * Current_Gain;
  ClarkeTransformaion(&Iabc, &Ialpha_Beta);
  ParkTransformaion(&Ialpha_Beta, Theta, &Cerrent_Udq);//帕克 克拉克变换获取当前的Iq值
  LowPassFilter_Calc(&Id_filter, Cerrent_Udq.x1);
  LowPassFilter_Calc(&Iq_filter, Cerrent_Udq.x2);
  LowPassFilter_Calc(&speed, encoder.Speed);


  Theta=_normalizeAngle(SENSOR_DIRECTION * POLE_PAIR * encoder.Angle
                          - ZeroElectricAngle);

  /*闭环*/
  Target_Udq.x2 = pid_calc(&Speed_PID, speed.previous_output, TargetSpeed);
  Udq.x1 = 0;//pid_calc(&Id_PID,Cerrent_Udq.x1,0);
  Udq.x2 = 0.5;//pid_calc(&Iq_PID,Cerrent_Udq.x2,Target_Udq.x2);

  /*输出*/
  SetTorque(&Target_Udq, Theta);
//  timestamp = now_us;

  /*调试*/
//  printf("%f,%f,%f,%f\n",SpeedFilter.out,  Target_Udq.x2, encoder.Angle,ZeroElectricAngle);
  printf("%f,%f,%f\n",encoder.Angle,speed.previous_output, Target_Udq.x2);
//  printf("%f,%f,%f\n",speed.previous_output, encoder.Angle, Target_Udq.x2);
}

/**
 * @brief  Clarke线性变换，将一个三相电流向量分解为二相正交向量
 *         基本公式为：
 *                      | I_alpha |       |  1    -1/2     -1/2     |       | Ia |
 *                      | I_beta  |   =   |  0   3^0.5/2  -3^0.5/2  |   x   | Ib |
 *                                                                          | Ic |
 * @param  input 被变换的三相电流向量，是一个三维的向量
 * @param  output 变换得到的二相正交电流向量，是一个二维的向量
 * @retval none
 */
void FOC::ClarkeTransformaion(Vector3 *Input, Vector2 *Output){
  Vector2* current_ab = Output;
  current_ab->x1 = (Input->x1 - Input->x2 * 0.5f         - Input->x3 * 0.5f);
  current_ab->x2 = (0.0f      + Input->x2 * SQRT3_2_NUM  - Input->x3 * SQRT3_2_NUM);
}

/**
 * @brief  反Clarke线性变换，将一个二相正交分解为三相电流向量
 *         基本公式为：
 *                      | I_a |       |  1        0     |       | Ialpha |
 *                      | I_b  |   =  |  -1/2   Sqrt3/2    |   x   | Ibeta |
 *                      | Ic |        |  1/2     Sqrt3/2   |
 * @param  input 二相正交电流向量，是一个二维的向量
 * @param  output 变换的三相电流向量，是一个三维的向量
 * @retval none
 */
void FOC::InverseClarkeTransformaion(Vector2 *Input,  Vector3 *Output){
  Vector3* current_abc = Output;
  current_abc->x1 = Input->x1 + 4.0f;
  current_abc->x2 = -0.5f * Input->x1 + SQRT3_2_NUM * Input->x2 + 4.0f;
  current_abc->x3 = 0.5f * Input->x1 + SQRT3_2_NUM * Input->x2 + 4.0f;
}
/**
 * @brief  Park线性变换，将一个二相静止坐标系变换到二相旋转坐标系，实质上只是一个旋转矩阵。
 *         基本公式为：
 *                      | I_d |   =   |  cos(theta)  sin(theta)  |   x   | I_alpha | 
 *                      | I_q |       | -sin(theta)  cos(theta)  |       | I_beta  |
 * 
 * @param  input 被变换的二相静止电流向量
 * @param  theta 角度
 * @param  output 变换得到的二相旋转电流向量
 * @retval none
 */
void FOC::ParkTransformaion(Vector2 *Input, fp32 Theta, Vector2 *Output){
  Vector2* current_dq = Output;
  current_dq->x1 = Input->x1 * CosByLut(Theta) + Input->x2 * SinByLut(Theta);
  current_dq->x2 = -Input->x1 * SinByLut(Theta) + Input->x2 * CosByLut(Theta);
}

/**
 * @brief  Park反变换，将一个二相旋转坐标系变换到二相静止坐标系，实质上只是一个逆向的旋转矩阵。
 *         基本公式为：
 *                      | U_alpha |   =   |  cos(theta) -sin(theta)  |   x   | U_d | 
 *                      | U_beta  |       |  sin(theta)  cos(theta)  |       | U_q |
 */
void FOC::InverseParkTransformaion(Vector2 *Input, fp32 Theta, Vector2 *Output){
  Vector2* UalphaBeta = Output;
  UalphaBeta->x1 = Input->x1 * CosByLut(Theta) - Input->x2 * SinByLut(Theta);
  UalphaBeta->x2 = Input->x1 * SinByLut(Theta) + Input->x2 * CosByLut(Theta);
}
/**
 * @brief  SVPWM计算函数、根据静止二相坐标系下的电压向量，计算出PWM中心对齐模式下U、V、W三相的接通时间，即脉冲宽度。
 *         该函数需要一个标记PWM周期长度的宏定义 SVPWM_PERIOD
 *         注意：仅仅适用于PWM中心对齐模式的应用场景。
 * @param  input 被变换的二相静止电压向量
 * @param  modulation SVPWM调制度，表示了期望输出的大小,0 < modulation < 1
 * @param  output 输出的SVPWM三相脉冲宽度（PWM中心对齐模式下）
 * @retval 
 */
void FOC::SVPWM(Vector2 *Input, float Modulation, Vector3 *Output){
    /* 判断扇区 */
    float alpha = Input->x1;
    float beta  = Input->x2;
    float beta_div_sqrt3 = beta * SQRT3_3_NUM;  /* 计算 beta/sqrt(3) */
    uint8_t sector;

    /* beta > 0 在 1、2、3 扇区 */
    /* beta < 0 在 4、5、6 扇区 */
    /* alpha 与 -beta/sqrt(3) 和 beta/sqrt(3) 的关系可以区分 1 4、2 5、3 6 扇区 */
    if(beta>=0){
        if(alpha>beta_div_sqrt3){
            sector = 1;
        }else{
            if(alpha>-beta_div_sqrt3){
                sector = 2;
            }else{
                sector = 3;
            }
        }
    }else{
        if(alpha>=-beta_div_sqrt3){
            sector = 6;
        }else{
            if(alpha>beta_div_sqrt3){
                sector = 5;
            }else{
                sector = 4;
            }
        }
    }

    /* 计算电压基矢量 */
    /* 计算SVPWM脉冲宽度：先计算零矢量T7，然后依次累加出另外两个基矢量 */
    float U_1, U_2;
    float Tu, Tv, Tw;
    switch (sector) {
        /* 扇区1  变换矩阵为             */
        /*        |  1  -3^0.5/3    |    */
        /*        |  0   3^0.5*2/3  |    */
        case 1:
            U_1 = alpha - beta_div_sqrt3;   /* U 0 0 */
            U_2 = 2.0f * beta_div_sqrt3;    /* U V 0 */
            Tw = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tv = Modulation * U_2 * SVPWM_PERIOD + Tw;
            Tu = Modulation * U_1 * SVPWM_PERIOD + Tv;
            break;

        /* 扇区2  变换矩阵为             */
        /*        |  1   3^0.5/3    |    */
        /*        | -1   3^0.5/3    |    */
        case 2:
            U_1 = alpha + beta_div_sqrt3;  /* U V 0 */
            U_2 = -alpha + beta_div_sqrt3; /* 0 V 0 */
            Tw = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tu = Modulation * U_1 * SVPWM_PERIOD + Tw;
            Tv = Modulation * U_2 * SVPWM_PERIOD + Tu;
            break;

        /* 扇区3  变换矩阵为             */
        /*        |  0   3^0.5*2/3  |    */
        /*        | -1  -3^0.5/3    |    */
        case 3:
            U_1 = 2.0f * beta_div_sqrt3;   /* 0 V 0 */
            U_2 = -alpha - beta_div_sqrt3; /* 0 V W */
            Tu = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tw = Modulation * U_2 * SVPWM_PERIOD + Tu;
            Tv = Modulation * U_1 * SVPWM_PERIOD + Tw;
            break;

        /* 扇区4  变换矩阵为             */
        /*        | -1   3^0.5/3    |    */
        /*        |  0  -3^0.5*2/3  |    */
        case 4:
            U_1 = -alpha + beta_div_sqrt3; /* 0 V W */
            U_2 = -2.0f * beta_div_sqrt3;  /* 0 0 W */
            Tu = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tv = Modulation * U_1 * SVPWM_PERIOD + Tu;
            Tw = Modulation * U_2 * SVPWM_PERIOD + Tv;
            break;

        /* 扇区5  变换矩阵为             */
        /*        | -1  -3^0.5/3    |    */
        /*        |  1  -3^0.5/3    |    */
        case 5:
            U_1 = -alpha - beta_div_sqrt3; /* 0 0 W */
            U_2 = alpha - beta_div_sqrt3;  /* U 0 W */
            Tv = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tu = Modulation * U_2 * SVPWM_PERIOD + Tv;
            Tw = Modulation * U_1 * SVPWM_PERIOD + Tu;
            break;

        /* 扇区6  变换矩阵为             */
        /*        |  0  -3^0.5*2/3  |    */
        /*        |  1   3^0.5/3    |    */
        case 6:
            U_1 = -2.0f * beta_div_sqrt3; /* U 0 W */
            U_2 = alpha + beta_div_sqrt3; /* U 0 0 */
            Tv = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tw = Modulation * U_1 * SVPWM_PERIOD + Tv;
            Tu = Modulation * U_2 * SVPWM_PERIOD + Tw;
            break;

        default:
            U_1 = 0.0f; /* 0 0 0 */
            U_2 = 0.0f; /* 0 0 0 */
            Tu = (1.0f - Modulation * (U_1 + U_2)) * SVPWM_PERIOD * 0.5f;
            Tv = Modulation * U_1 * SVPWM_PERIOD + Tu;
            Tw = Modulation * U_2 * SVPWM_PERIOD + Tv;
            break;
    }

    Output->x1 = Tu;
    Output->x2 = Tv;
    Output->x3 = Tw;
}

void FOC:: Encoder_Update(){
  encoder.Angle = ReadFromSensor(READ_ANGLE_VAL_CMD);//读出值范围为360的
  encoder.Angle *= PI / 180.0f;
  encoder.Speed=-Get_velocity();
  encoder.Temp  = ReadFromSensor(READ_TEMP_CMD);
}

void FOC::SetPWM(Vector3 Uabc) {
  Uabc.x1 = Constrain(Uabc.x1, 0, SVPWM_PERIOD);//限幅
  Uabc.x2 = Constrain(Uabc.x2, 0, SVPWM_PERIOD);
  Uabc.x3 = Constrain(Uabc.x3, 0, SVPWM_PERIOD);
  htim1.Instance->CCR1 = (uint16_t) Uabc.x1;
  htim1.Instance->CCR2 = (uint16_t) Uabc.x2;
  htim1.Instance->CCR3 = (uint16_t) Uabc.x3;
//  printf("%d,%d,%d\n", (uint16_t)Uabc.x1, (uint16_t)Uabc.x2, (uint16_t)Uabc.x3);
}

void FOC::SetTorque(Vector2 *udq, fp32 angle_el) {
  InverseParkTransformaion(udq, Theta, &Ualpha_Beta);
  SVPWM(&Ualpha_Beta, MODULATION,&Svpwm_abc);
  SetPWM(Svpwm_abc);
}

void FOC::FOC_electrical_angle_calibration(){
  Vector2 Temp_dq;
  Temp_dq.x1 = 8.0f;        Temp_dq.x2 = 0.0f;
  SetTorque(&Temp_dq, 0);
  HAL_Delay(200);
  Encoder_Update();
  Last_Angle = encoder.Angle;
  ZeroElectricAngle = _normalizeAngle(SENSOR_DIRECTION * POLE_PAIR * encoder.Angle
                                          - ZeroElectricAngle);
  Temp_dq.x1 = 0.0f;        Temp_dq.x2 = 0.0f;
  SetTorque(&Temp_dq, 0);
}
float FOC::Get_velocity() const {
  static uint32_t pre_tick;
  static float last_mechanical_angle = 0;
  float ts = (float) (HAL_GetTick() - pre_tick) * 1e-3f;
  if (ts < 1e-3) ts = 1e-3;
  pre_tick = HAL_GetTick();

  float delta_angle = encoder.Angle - last_mechanical_angle;
  last_mechanical_angle = encoder.Angle;
  if (abs(delta_angle) > PI) {
    if (delta_angle > 0) {
      delta_angle -= _2PI;
    } else {
      delta_angle += _2PI;
    }
  }
  return delta_angle / ts;
}

void FOC::Debug() {
//  printf("%f,%f,%f,%f\n",Iabc.x1,Iabc.x2, Iabc.x3, Iq_filter.out);
}
/*Temp:
//  unsigned long now_us = micros();
//  Ts = (float )(now_us - timestamp) * 1e-6f;
//  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
//  shaft_angle = _normalizeAngle(shaft_angle + Ts * Target);
//  Theta=_electricalAngle(shaft_angle, POLE_PAIR);
//  encoder.Speed = delta_angle / Ts;
//  Last_Angle = encoder.Angle;

 */




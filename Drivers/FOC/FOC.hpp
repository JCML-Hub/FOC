#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Libray Include */
#include "main.h"
#include "function.h"
#include "utilities.h"
#include "dma.h"

/* Defines */
#define MODULATION  (1.0f)      /* SVPWM的周期总长 当前值为5600 */
#define POLE_PAIR     11
#define SENSOR_DIRECTION  (-1)
#define SQRT3_NUM     1.73205080757f
#define SQRT3_2_NUM   0.86602540378f
#define SQRT3_3_NUM   0.57735026919f
#define Current_Gain  0.00157704454f


/* Struct Types */
typedef struct{
    fp32 x1, x2;
} Vector2;

typedef struct{
    fp32 x1, x2, x3;     // 3 Vector components
} Vector3;
typedef struct {
    float Angle;
    float Speed;
    float Temp;
} Encoder;

class FOC{
    private:
        /* data */
        first_order_filter_type_t SpeedFilter;
        first_order_filter_type_t AngleFilter;
        Kalman Speed;
        LowPassFilter speed;
        LowPassFilter Id_filter;
        LowPassFilter Iq_filter;


        fp32 Ts;
        fp32    Theta;          // 相位角
        Vector2 Target_Udq;
        Vector2 Udq;
        Vector2 Cerrent_Udq;
        Vector2 Ualpha_Beta;
        Vector2 Ialpha_Beta;
        Vector3 Iabc;
        uint16_t ADC_Value[4]={0};
        pid_type_t Iq_PID;
        pid_type_t Id_PID;
        pid_type_t Speed_PID;
        pid_type_t Angle_PID;

        void Encoder_Update();
        void Debug();
    public:
        Encoder encoder;
        fp32 ZeroElectricAngle;
        Vector3 Svpwm_abc;

        void Init();
        static void ClarkeTransformaion(Vector3 *Input, Vector2 *Output);
        void InverseClarkeTransformaion(Vector2 *Input,  Vector3 *Output);
        void ParkTransformaion(Vector2 *Input,fp32 Theta, Vector2 *Output);
        static void InverseParkTransformaion(Vector2 *Input,fp32 Theta, Vector2 *Output);
        void SVPWM(Vector2 *Input, float Modulation, Vector3 *Output);
        void Update();
        static void SetPWM(Vector3 Uabc);
        void FOC_electrical_angle_calibration();
        void SetTorque(Vector2 *udq, fp32 angle_el);
        float Get_velocity() const;
};

__weak void ADC_DMACallback(struct __DMA_HandleTypeDef *hdma);
#ifdef __cplusplus
}
#endif
#endif

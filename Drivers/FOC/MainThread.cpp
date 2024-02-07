#include "MainThread.h"
#include "Function.h"
#include "Time_utils.h"
#include "FOC.hpp"
#include "utilities.h"

 FOC foc;

void Main_Thread(void){
  foc.Init();
	while(true){
    foc.Update();
//    foc.SetPWM(foc.Svpwm_abc);
//    printf("%d\n", TIM1->CNT);
//     printf("Data:%f,%f,%f\r\n", foc.encoder.Angle, foc.encoder.Speed, foc.encoder.Temp);
	}
}
/*
//		unsigned long now_us = micros();
//		float Ts = (now_us - timestamp) * 1e-6f;
//		if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
//		shaft_angle = _normalizeAngle(shaft_angle + Ts * Target);
//    _angle=_electricalAngle(shaft_angle, 12);
//		Printf("%f\n", SinByLut(_angle));
//		timestamp = now_us;
 * */

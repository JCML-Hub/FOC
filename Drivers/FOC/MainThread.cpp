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

//void ADC_DMACallback(struct __DMA_HandleTypeDef *hdma) {
//  foc.filter();
//}

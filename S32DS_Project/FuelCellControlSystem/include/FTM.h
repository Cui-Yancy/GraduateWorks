#ifndef FTM_H_
#define FTM_H_

#include "s32k144.h"

#define FTM0_PWMChannel     7U
#define PWMFAN_MAX          9000  //90%
#define PWMFAN_MIN          700    //7%

#define FTM1INTPRI          5

void FTM0_Init(void);
void FTM0_UpdatePWM(uint8_t CH,uint16_t Duty);

//void FTM1_StartCounter(void);
//void FTM1_CH0_GeneratePause(uint32_t Time);
void FTM1_Channel0_OC_IRQHandler(void);
//void FTM1_Init(void);
//void FTM1_CH0_OC_init(void);
void Delay(uint32_t Time);

void FTM1_Init_Fixed(void);
void FTM1_CH0_GeneratePause_Fixed(uint32_t DelayTime,uint32_t DurationTime);

#endif /* FTM_H_ */

#ifndef ADC0_H_
#define ADC0_H_

#include "s32k144.h"
#include "CAN.h"

#define ADC0_Ch0        0U
#define ADC0_ResultNum  100U
#define ADC0_MAXValue   4095U
#define ADC0_ReferenceV 4.99f
#define AssistR         996U

extern volatile uint16_t ADC0_Result[ADC0_ResultNum];

void ADC0_Init(void);
void ADC0_CalculateT(double *Temperature, CANMessage *message);
uint16_t ResultFilter(volatile uint16_t* ResultArray, uint16_t ArraySize);

#endif /* ADC0_H_ */

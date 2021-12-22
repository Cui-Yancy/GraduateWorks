
#ifndef ADC1_H_
#define ADC1_H_

#include "s32k144.h"
#include "CAN.h"

#define ADC1_Channel    0U
#define ADC1_ResultNum  100U
#define ADC1_MAXValue   4095U
#define ADC1_ReferenceV 4.99f
//#define AssistR       996U

#define ADC1_V_Channel  1U  //ADC1_SE6  PTD4
#define ADC1_V_ResultNum    100U

extern volatile uint16_t ADC1_Result[ADC1_ResultNum],ADC1_V_Result[ADC1_V_ResultNum];

void ADC1_Init(void);
void ADC1_CalculateI(double *I, CANMessage *message);
void ADC1_CalculateV(double *V, CANMessage *message);

#endif /* ADC1_H_ */

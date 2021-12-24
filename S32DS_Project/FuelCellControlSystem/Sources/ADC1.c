#include "ADC1.h"
#include "adc_driver.h"
#include "adConv2.h"
#include "ADC0.h"
#include "CAN.h"

volatile uint16_t ADC1_Result[ADC1_ResultNum]={0,};
volatile uint16_t ADC1_V_Result[ADC1_V_ResultNum]={0,};

/*ADC1通道0设置为软件触发，初始化时将其通道连接至电流采集
 * 在DMA完成100组数据传输后，将改通道连接到电压采集
 * */
void ADC1_Init(void){
    ADC_DRV_Reset(INST_ADCONV2);
    ADC_DRV_AutoCalibration(INST_ADCONV2);
    ADC_DRV_ConfigConverter(INST_ADCONV2, &adConv2_ConvConfig0);
    //ADC_DRV_ConfigChan(INST_ADCONV2, ADC1_Channel, &adConv2_ChnConfig0);
    //ADC_DRV_ConfigChan(INST_ADCONV2, ADC1_V_Channel/*uint8_t chanIndex*/, &adConv2_ChnConfig1);
}

void ADC1_CalculateI(double *I, CANMessage *message){
    uint16_t ADC1_Average = 0;
    double k = 32.993;
    static uint16_t ADC_At0A = 2042;
    ADC1_Average = ResultFilter(ADC1_Result,ADC1_ResultNum);
    if(ADC1_Average<2050 && ADC1_Average>2030){
        ADC_At0A = ADC1_Average;
    }

    message->ID = ID_I;
    message->isExtendFrame = true;
    message->MessageArry[0] = (uint8_t)((ADC1_Average<<8)>>8);
    message->MessageArry[1] = (uint8_t)(ADC1_Average>>8);    //高8位
    message->MessageArry[2] = 0;
    message->MessageArry[3] = 0;
    message->MessageArry[4] = 0;
    message->MessageArry[5] = 0;
    message->MessageArry[6] = 0;
    message->MessageArry[7] = 0;

    *I = (ADC1_Average-ADC_At0A) / k ;
    //*I = ADC1_Average / 100.0 ;
}

void ADC1_CalculateV(double *V, CANMessage *message){
    uint16_t ADC1_V_Average = 0;
    double k = 1.1844,V_Temp,b = 0.00;
    uint32_t V_x10000;
    ADC1_V_Average = ResultFilter(ADC1_V_Result,ADC1_V_ResultNum);

    V_Temp = k*25*ADC1_V_Average / 4095 - b;
    *V = V_Temp;
    V_x10000 = 10000*V_Temp;

    message->ID = ID_V;
    message->isExtendFrame = true;
    message->MessageArry[0] = (uint8_t)((V_x10000 & 0x000000FF)>>0);
    message->MessageArry[1] = (uint8_t)((V_x10000 & 0x0000FF00)>>8);
    message->MessageArry[2] = (uint8_t)((V_x10000 & 0x00FF0000)>>16);
    message->MessageArry[3] = 0;
    message->MessageArry[4] = 0;
    message->MessageArry[5] = 0;
    message->MessageArry[6] = 0;
    message->MessageArry[7] = 0;
}

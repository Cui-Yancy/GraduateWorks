#include "ADC0.h"
#include "adc_driver.h"
#include "adConv1.h"
#include <stdio.h>
#include <math.h>
#include "CAN.h"

volatile uint16_t ADC0_Result[ADC0_ResultNum]={0,};

void ADC0_Init(void){
    ADC_DRV_Reset(INST_ADCONV1);
    ADC_DRV_AutoCalibration(INST_ADCONV1);
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);
    ADC_DRV_ConfigChan(INST_ADCONV1, ADC0_Ch0/*uint8_t chanIndex*/, &adConv1_ChnConfig0);
}

void ADC0_CalculateT(double *Temperature, CANMessage *message){
    uint16_t ADC0_Average = 0;
    ADC0_Average = ResultFilter(ADC0_Result,ADC0_ResultNum);

    

    //float V = ADC0_ReferenceV*ADC0_Average/(float)ADC0_MAXValue;
    //Printf("V = %.3fV\t",V);

    //double R = AssistR*ADC0_Average / (double)(ADC0_MAXValue - ADC0_Average);
    //*Temperature = (R-1000)/3.85;
    *Temperature = ADC0_Average*258.7/(4095-ADC0_Average)-259.74;
    //printf("T = %.2f\r\n",*Temperature);
    
    uint16_t num = *Temperature*10;
    message->ID = ID_T;
    message->isExtendFrame = true;
    message->MessageArry[0] = (uint8_t)((num<<8)>>8);
    message->MessageArry[1] = (uint8_t)(num>>8);    //高8位
    message->MessageArry[2] = 0;
    message->MessageArry[3] = 0;
    message->MessageArry[4] = 0;
    message->MessageArry[5] = 0;
    message->MessageArry[6] = 0;
    message->MessageArry[7] = 0;
}




//拉依达准则剔除无效数据
//1、计算原始数据和、原始数据均值
//2、计算原始数据标准差、方差
//3、将标准差落在3Sigma以外数据剔除，再求均值并返回
uint16_t ResultFilter(volatile uint16_t* ResultArray, uint16_t ArraySize) {
     uint16_t TempArray[ArraySize];//将原始数据保存，防止函数执行过程中被破坏
     //求原始数据和、均值
     uint32_t sum = 0;//数据和
     double Avr = 0;//原始均值
     int i = 0;
     for (i = 0; i < ArraySize; i++) {
        TempArray[i] = ResultArray[i];
        sum += TempArray[i];
     }
     Avr = (double)sum / ArraySize;

     //正态分布判断
//   uint16_t min = TempArray[0];
//   uint16_t max = TempArray[0];
//   for(i = 0;i<ArraySize;i++){
//       if(TempArray[i]>max)
//           max = TempArray[i];
//       if(TempArray[i]<min)
//           min = TempArray[i];
//   }
//   int Section_len = 1;
//   int group = (max-min)/Section_len+1;
//   //Printf("%d ", group);
//   int section[group];
//   for(i=0;i<group;i++){
//       section[i] = 0;
//   }
//   for(i=0;i<ArraySize;i++){
//       int temp = (TempArray[i]-min)/Section_len;
//       section[temp]++;
//   }
//   for(i=0;i<group;i++){
//       Printf("%d ", section[i]);
//   }
//   Printf("\n\r");

     //计算方差和标准差
     double Sigma = 0;//标准差
     double Sigma_2 = 0;//方差
     double temp_D = 0;//偏差
     for (i = 0; i < ArraySize; i++) {
          temp_D = (TempArray[i] - Avr);
          Sigma_2 += temp_D * temp_D;
     }
     Sigma_2 /= ArraySize;
     Sigma = sqrt(Sigma_2);

     //剔除无效数据，再算均值
     sum = 0;
     unsigned int Useless_Data = 0;//记录不合理数据个数
     double Sigma_triple = 3*Sigma;//3*Sigma作为判断标准
     for ( i = 0; i < ArraySize; i++) {
          //落在3D外，跳过，记录坏数据个数
          if (fabs(TempArray[i] - Avr) > Sigma_triple) {
           Useless_Data++;
           continue;
          }
          sum += TempArray[i];
     }
     //printf("Useless_Data = %d\t", Useless_Data);
     uint16_t Avr_real = (float)sum / (ArraySize - Useless_Data);
     return Avr_real;
}






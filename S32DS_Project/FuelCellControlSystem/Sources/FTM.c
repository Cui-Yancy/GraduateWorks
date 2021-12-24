#include "FTM.h"

#include "s32k144.h"
#include "interrupt_manager.h"

uint8_t NeedPause = false;
uint16_t PauseDurationTicks = 0;
uint16_t PauseCycle = 0;            //如果要产生比单个周期还要长的脉冲信号，需要让计数器多进几次中断后再结束脉冲

/* FTM0_CH7用作PWM,PTE7
 * 用于调节风扇转速
 * 默认频率为120Hz
 * 低频PWM相当于调节风扇供电电压
 *
 * */
//void FTM0_Init(void){
//    ftm_state_t ftmStateStruct;
//    FTM_DRV_Init(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_InitConfig,&ftmStateStruct/* ftm_state_t * state */);
//    FTM_DRV_InitPwm(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_PwmConfig);
//}

//4MHz,100Hz
void FTM0_Init(void) {
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;  /* Ensure clk disabled for config */
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b001)  /* Clock Src=1, 8 MHz SOSCDIV1_CLK */
                                |  PCC_PCCn_CGC_MASK;   /* Enable clock for FTM regs */
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;  /* Write protect to registers disabled (default) */
    FTM0->SC = 0x00800001;     /* Enable PWM channel 7 output*/
                                /* TOIE (Timer Overflow Interrupt Ena) = 0 (default) */
                                /* CPWMS (Center aligned PWM Select) = 0 (default, up count) */
                                /* CLKS (Clock source) = 0 (default, no clock; FTM disabled) */
                                /* PS (Prescaler factor) = 7. Prescaler = 128 */
    FTM0->COMBINE = 0x00000000;/* FTM mode settings used: DECAPENx, MCOMBINEx, COMBINEx=0  */
    FTM0->POL = 0x00000000;    /* Polarity for all channels is active high (default) */
    FTM0->MOD = 40000-1 ;     /* FTM1 counter final value (used for PWM mode) */
                                /* FTM1 Period = MOD-CNTIN+0x0001 ~= 62500 ctr clks  */
                                /* 8MHz /128 = 62.5kHz ->  ticks -> 1Hz */
    FTM0->CONTROLS[7].CnSC = 0x00000028;  /* FTM0 ch1: edge-aligned PWM, low true pulses */
                                        /* CHIE (Chan Interrupt Ena) = 0 (default) */
                                        /* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM*/
                                        /* ELSB:ELSA (chan Edge/Level Select)=0b10, low true */
    FTM0->CONTROLS[7].CnV =  0;       /* FTM0 ch1 compare value (~75% duty cycle) */

    FTM0->SC |= FTM_SC_CLKS(3);
}

//@pram Duty:0~10000,代表0%~100%,500代表5%
// void FTM0_UpdatePWM(uint8_t CH,uint16_t Duty){
//     uint16_t DutyCycle = (uint16_t)((uint32_t)FTM_MAX_DUTY_CYCLE*Duty/10000);
//     FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, CH, FTM_PWM_UPDATE_IN_DUTY_CYCLE, DutyCycle, 0U, true);
// }

void FTM0_UpdatePWM(uint8_t CH,uint16_t Duty){
    uint16_t MOD = FTM0->MOD;
    FTM0->CONTROLS[CH].CnV =  MOD*(Duty)/10000.0;
    //while(FTM0->CNT != FTM0->CNTIN);
}

void FTM1_Channel0_OC_IRQHandler(void){
    static uint16_t CurrentCycle = 0;
    //uint16_t Count = 0;
    if(NeedPause){//此时已经拉高
        CurrentCycle++;
    }else{//此时已经拉低
        //FTM1->CONTROLS[0].CnSC &= ~FTM_CnSC_CHIE_MASK;  //关闭中断
        FTM1->SC &= ~FTM_SC_PWMEN0_MASK;    //关闭PWM通道输出
        INT_SYS_DisableIRQ(FTM1_Ch0_Ch1_IRQn);  //关闭中断
        FTM1->MODE |= FTM_MODE_WPDIS_MASK;
        FTM1->SC &= ~FTM_SC_CLKS(3);        //关闭FTM
    }
    if((CurrentCycle-1)==PauseCycle){
        FTM1->CONTROLS[0].CnV = (FTM1->CONTROLS[0].CnV+PauseDurationTicks)%(FTM1->MOD+1);
        //Count = FTM1->CNT;
        //while(FTM1->CNT<=Count+1);
        NeedPause = false;
        CurrentCycle = 0;
        //Delay(WaitToChange);
        FTM1->MODE |= FTM_MODE_WPDIS_MASK;
        FTM1->CONTROLS[0].CnSC |= (1<<2);  //下一次拉低
    }
    FTM1->CONTROLS[0].CnSC &= (~FTM_CnSC_CHF_MASK);         //清除标志位
}



void Delay(uint32_t Time){
    while(Time--);
}

uint32_t Frequency = 0;
/*FTM1初始化函数
 * 晶振频率：8MHz，2分频
 * 最大值：40000-1，最大时间长度：10ms,分辨率：1/4M 0.25us
 * */
void FTM1_Init_Fixed(void) {
    PCC->PCCn[PCC_FTM1_INDEX] &= ~PCC_PCCn_CGC_MASK;  /* Ensure clk disabled for config */
    PCC->PCCn[PCC_FTM1_INDEX] |= PCC_PCCn_PCS(0b001)  /* Clock Src=1, 8 MHz SOSCDIV1_CLK */
                            |  PCC_PCCn_CGC_MASK;   /* Enable clock for FTM regs */
    FTM1->MODE |= FTM_MODE_WPDIS_MASK;  /* Write protect to registers disabled (default) */
    FTM1->SC = 0x00000001;     //禁用定时器翻转中断，分频为64，计时器禁用，无通道使能

    uint8_t Count = FTM1->SC&0x7,i=0;
    Frequency = 8000000;
    for(i=0;i<Count;i++)
        Frequency /= 2;

    FTM1->COMBINE = 0x00000000;/* FTM mode settings used: DECAPENx, MCOMBINEx, COMBINEx=0  */
    FTM1->POL = 0x000000FF;    //极性为低电平
    FTM1->MOD = 40000-1 ;     //周期65.5ms

    FTM1->CONTROLS[0].CnSC = 0x00000058;    //中断使能，拉高

    //FTM1_CH0_OC_init();
    INT_SYS_InstallHandler(FTM1_Ch0_Ch1_IRQn,&FTM1_Channel0_OC_IRQHandler,(isr_t *)0);
    INT_SYS_SetPriority(FTM1_Ch0_Ch1_IRQn,FTM1INTPRI);
    

    //FTM1_StartCounter();
    //FTM1->SC |= FTM_SC_CLKS(3);
}

/*用于产生脉冲
 * DurationTime:us
 * */
void FTM1_CH0_GeneratePause_Fixed(uint32_t DelayTime,uint32_t DurationTime){
    uint32_t CurrentTime = (FTM1->CNT+20)%(FTM1->MOD+1);     //+1时更新CnV,+2时才能识别
    uint16_t DelayTimeTicks = (uint16_t)(((uint64_t)Frequency*DelayTime)/1000000);
    uint32_t NeedTicks = (uint32_t)((uint64_t)DurationTime*Frequency/1000000);
    PauseCycle = (uint16_t)(NeedTicks/(FTM1->MOD+1));
    PauseDurationTicks = (uint16_t)(NeedTicks%(FTM1->MOD+1));
    if(PauseDurationTicks==0)
    {
        PauseCycle--;
    }

    FTM1->CONTROLS[0].CnV = (CurrentTime+DelayTimeTicks)%(FTM1->MOD+1);
    
    FTM1->MODE |= FTM_MODE_WPDIS_MASK;
    FTM1->CONTROLS[0].CnSC &= ~(1<<2);  //下一次拉高
    INT_SYS_EnableIRQ(FTM1_Ch0_Ch1_IRQn);
    FTM1->MODE |= FTM_MODE_WPDIS_MASK;
    FTM1->SC |= FTM_SC_PWMEN0(1);   //使能PWM通道输出
    FTM1->SC |= FTM_SC_CLKS(3);
    NeedPause = true;
}


#include "S32K144.h"
#include "DMA.h"
#include "ADC0.h"
#include "ADC1.h"
#include "interrupt_manager.h"
#include "adc_driver.h"
#include "adConv2.h"

/*先使能ADC0的温度采集DMA，传输100组数据后进入中断，切换到ADC1进行电流采集
 * */
void DMA_Init(void){
    DMA_CH0_ADC0_Init();
    DMA_CH4_ADC1_Init();
    DMA_CH3_ADC1_V_Init();

    INT_SYS_InstallHandler(DMA0_IRQn,&DMA_CH0_IRQHandler,(isr_t *)0);
    INT_SYS_SetPriority(DMA0_IRQn,DMAINTPRI);
    INT_SYS_EnableIRQ(DMA0_IRQn);

    INT_SYS_InstallHandler(DMA4_IRQn,&DMA_CH4_IRQHandler,(isr_t *)0);
    INT_SYS_SetPriority(DMA4_IRQn,DMAINTPRI);
    INT_SYS_EnableIRQ(DMA4_IRQn);

    INT_SYS_InstallHandler(DMA3_IRQn,&DMA_CH3_IRQHandler,(isr_t *)0);
    INT_SYS_SetPriority(DMA3_IRQn,DMAINTPRI);
    INT_SYS_EnableIRQ(DMA3_IRQn);

    DMA_ChannelReqEnable(DMA_ADC0_T_Channel0);
}

void DMA_CH0_ADC0_Init(void){
    DMA_TCD_Config_T DMA_TCD_Configuration;

    DMA_TCD_Configuration.SAddr     =   (uint32_t)&(ADC0->R[ADC0_Ch0]);
    DMA_TCD_Configuration.SOffset   =   0;
    DMA_TCD_Configuration.SMode     =   0;
    DMA_TCD_Configuration.SSize     =   bit_16;
    DMA_TCD_Configuration.SLastAddrOffset       =   0;

    DMA_TCD_Configuration.DAddr     =   (uint32_t)&ADC0_Result[0];
    DMA_TCD_Configuration.DOffset   =   2;
    DMA_TCD_Configuration.DMode     =   0;
    DMA_TCD_Configuration.DSize     =   bit_16;
    DMA_TCD_Configuration.DLastAddrAdjust       =   -ADC0_ResultNum*2;

    DMA_TCD_Configuration.MinorLoopTransBytes   =   2;
    DMA_TCD_Configuration.MajorIterCount        =   ADC0_ResultNum;
    DMA_TCD_Configuration.StartMajorIterCount   =   ADC0_ResultNum;
    DMA_TCD_Configuration.DisableReq            =   true;   //主循环完成后，关闭通道的请求，如果需要再次打开，需要重新使胿
    DMA_TCD_Configuration.EnableHalfInt         =   false;
    DMA_TCD_Configuration.EnableMajorInt        =   true;   //主循环完成时产生中断
    DMA_TCD_Configuration.StartChan             =   false;

    DMAMUX_Init_Yancy(DMA_ADC0_T_Channel0,ADC0_Req);
    DMA_TCD_Init(DMA_ADC0_T_Channel0,DMA_TCD_Configuration);
    //DMA_ChannelReqEnable(DMA_ADC0_T_Channel0);
}

void DMA_CH4_ADC1_Init(void){
    DMA_TCD_Config_T DMA_TCD_Configuration;

    DMA_TCD_Configuration.SAddr     =   (uint32_t)&(ADC1->R[ADC1_Channel]);
    DMA_TCD_Configuration.SOffset   =   0;
    DMA_TCD_Configuration.SMode     =   0;
    DMA_TCD_Configuration.SSize     =   bit_16;
    DMA_TCD_Configuration.SLastAddrOffset       =   0;

    DMA_TCD_Configuration.DAddr     =   (uint32_t)&ADC1_Result[0];
    DMA_TCD_Configuration.DOffset   =   2;
    DMA_TCD_Configuration.DMode     =   0;
    DMA_TCD_Configuration.DSize     =   bit_16;
    DMA_TCD_Configuration.DLastAddrAdjust       =   -ADC1_ResultNum*2;

    DMA_TCD_Configuration.MinorLoopTransBytes   =   2;
    DMA_TCD_Configuration.MajorIterCount        =   ADC1_ResultNum;
    DMA_TCD_Configuration.StartMajorIterCount   =   ADC1_ResultNum;
    DMA_TCD_Configuration.DisableReq            =   true;
    DMA_TCD_Configuration.EnableHalfInt         =   false;
    DMA_TCD_Configuration.EnableMajorInt        =   true;
    DMA_TCD_Configuration.StartChan             =   false;

    DMAMUX_Init_Yancy(DMA_ADC1_I_Channel4,ADC1_Req);
    DMA_TCD_Init(DMA_ADC1_I_Channel4,DMA_TCD_Configuration);
    //DMA_ChannelReqEnable(DMA_ADC1_I_Channel4);
}

void DMA_CH3_ADC1_V_Init(void){
    DMA_TCD_Config_T DMA_TCD_Configuration;

    DMA_TCD_Configuration.SAddr     =   (uint32_t)&(ADC1->R[ADC1_Channel]);
    DMA_TCD_Configuration.SOffset   =   0;
    DMA_TCD_Configuration.SMode     =   0;
    DMA_TCD_Configuration.SSize     =   bit_16;
    DMA_TCD_Configuration.SLastAddrOffset       =   0;

    DMA_TCD_Configuration.DAddr     =   (uint32_t)&ADC1_V_Result[0];
    DMA_TCD_Configuration.DOffset   =   2;
    DMA_TCD_Configuration.DMode     =   0;
    DMA_TCD_Configuration.DSize     =   bit_16;
    DMA_TCD_Configuration.DLastAddrAdjust       =   -ADC1_V_ResultNum*2;

    DMA_TCD_Configuration.MinorLoopTransBytes   =   2;
    DMA_TCD_Configuration.MajorIterCount        =   ADC1_V_ResultNum;
    DMA_TCD_Configuration.StartMajorIterCount   =   ADC1_V_ResultNum;
    DMA_TCD_Configuration.DisableReq            =   true;
    DMA_TCD_Configuration.EnableHalfInt         =   false;
    DMA_TCD_Configuration.EnableMajorInt        =   true;
    DMA_TCD_Configuration.StartChan             =   false;

    DMAMUX_Init_Yancy(DMA_ADC1_V_Channel3,ADC1_Req);
    DMA_TCD_Init(DMA_ADC1_V_Channel3,DMA_TCD_Configuration);
    //DMA_ChannelReqEnable(DMA_ADC1_I_Channel4);
}


/*===========================================
 * 函数功能：初始化DMA模块，使能时钟，给DMA通道配置触发源
 * 触发源编号可通过datasheet附件--DMA_Interrupt_mapping查找
 * ==========================================*/
void DMAMUX_Init_Yancy(uint8_t Channel,TrigSource_T TrigSource){
  PCC->PCCn[PCC_DMAMUX_INDEX] |=   PCC_PCCn_CGC_MASK;
  DMAMUX->CHCFG[Channel]       =   0;
  DMAMUX->CHCFG[Channel]       =   DMAMUX_CHCFG_SOURCE(TrigSource)|
                                   DMAMUX_CHCFG_ENBL_MASK;
}

/*===========================================
 * 函数功能：配置DMA_TCD部分
 * 主要描述DMA主次循环，默认无通道链接
 *===========================================*/
void DMA_TCD_Init (uint8_t Channel,DMA_TCD_Config_T TCD){
    DMA->TCD[Channel].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile)TCD.SAddr);
    DMA->TCD[Channel].SOFF         = DMA_TCD_SOFF_SOFF(TCD.SOffset);
    DMA->TCD[Channel].ATTR         = DMA_TCD_ATTR_SMOD(TCD.SMode)  |
                                     DMA_TCD_ATTR_SSIZE(TCD.SSize) |
                                     DMA_TCD_ATTR_DMOD(TCD.DMode)  |
                                     DMA_TCD_ATTR_DSIZE(TCD.DSize);
    DMA->TCD[Channel].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(TCD.MinorLoopTransBytes);   //具体使用哪一个寄存器还需再查，pg346
    DMA->TCD[Channel].SLAST        = DMA_TCD_SLAST_SLAST(TCD.SLastAddrOffset);
    DMA->TCD[Channel].DADDR        = DMA_TCD_DADDR_DADDR(TCD.DAddr);
    DMA->TCD[Channel].DOFF         = DMA_TCD_DOFF_DOFF(TCD.DOffset);
    DMA->TCD[Channel].CITER.ELINKNO= DMA_TCD_CITER_ELINKNO_CITER(TCD.MajorIterCount) |
                                     DMA_TCD_CITER_ELINKNO_ELINK(0);                     /*通道连接禁用 */
    DMA->TCD[Channel].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(TCD.DLastAddrAdjust);
    DMA->TCD[Channel].CSR          = DMA_TCD_CSR_START(TCD.StartChan)           |
                                     DMA_TCD_CSR_INTMAJOR(TCD.EnableMajorInt)   |
                                     DMA_TCD_CSR_INTHALF(TCD.EnableHalfInt)     |
                                     DMA_TCD_CSR_DREQ(TCD.DisableReq)           |
                                     DMA_TCD_CSR_ESG(0)         |       /* 主循环完成后不需要修改TCD结构 */
                                     DMA_TCD_CSR_MAJORELINK(0)  |       /* 无通道链接 */
                                     DMA_TCD_CSR_ACTIVE(0)      |       /* 只读，表示DMA正在工作 */
                                     DMA_TCD_CSR_DONE(0)        |       /* 主循环完成标志，如果需要修改TCD或者通道链接，则需要写0 */
                                     DMA_TCD_CSR_MAJORLINKCH(0) |       /* 无通道连接 */
                                     DMA_TCD_CSR_BWC(0);                /* 不使用DMA缓冲 */
    DMA->TCD[Channel].BITER.ELINKNO= DMA_TCD_BITER_ELINKNO_BITER(TCD.StartMajorIterCount) |
                                     DMA_TCD_BITER_ELINKNO_ELINK(0);    /* 无通道连接 */
    //DMA->ERQ  =   1<<Channel;     /*使能该通道的DMA请求*/
}

/*========================================
 * 用于软件触发一次DMA通道
 * 如果设置为ADC触发时，不必软件触发，转换完成便会触发
 * =======================================*/
void DMA_StartChannel(uint8_t Channel){
    DMA->SSRT=Channel;
}

/*========================================
 * 取消剩余数据传输，等待当前次循环完成
 * */
void DMA_CancelTransfer(uint8_t Channel){
    DMA->ERQ &= ~(uint32_t)(1<<Channel);
    DMA->CR |= DMA_CR_CX_MASK;

}

void DMA_ChannelReqEnable(uint8_t Channel){
    DMA->ERQ    =   1<<Channel;     /*使能该通道的DMA请求*/
}

void DMA_CH0_IRQHandler(){//温度采集100组数据完成，下一个是电流采集
    ADC_DRV_ConfigChan(INST_ADCONV2, ADC1_Channel, &adConv2_ChnConfig0);
    DMA_ChannelReqEnable(DMA_ADC1_I_Channel4);
    DMA->INT |= 1<<0;
}

void DMA_CH4_IRQHandler(){//电流采集100组数据完成，下一个是电压采集
    ADC_DRV_ConfigChan(INST_ADCONV2, ADC1_Channel/*uint8_t chanIndex*/, &adConv2_ChnConfig1);
    DMA_ChannelReqEnable(DMA_ADC1_V_Channel3);
    DMA->INT |= 1<<4;
}

void DMA_CH3_IRQHandler(){//电压采集100组数据完成，下一个是温度采集
    DMA_ChannelReqEnable(DMA_ADC0_T_Channel0);
    DMA->INT |= 3<<1;
}




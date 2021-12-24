#ifndef DMA_H_
#define DMA_H_
#include "stdbool.h"
#include "FreeRTOSConfig.h"

#define DMA_ADC0_T_Channel0 0U
#define DMA_ADC1_I_Channel4 4U
#define DMA_ADC1_V_Channel3 3U

#define DMAINTPRI       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+3)

typedef enum{
    LPUART0_RXReq=2,
    LPUART0_TXReq=3,
    LPUART1_RXReq=4,
    LPUART1_TXReq=5,
    ADC0_Req=42,
    ADC1_Req=43,
    FlexCAN0_Req=54,
}TrigSource_T;
typedef enum{
    bit_8=0,
    bit_16=1,
    bit_32=2,
    byte_16=4,
    byte_32=5,
}TransferSize_T;
typedef struct{
    uint32_t SAddr;                 //源地址--注意为32位地址，可能需要使用&寻址
    uint16_t SOffset;               //源地址偏移量
    uint8_t SMode;                  //源模式--需要进一步验证，表示传输2^次方次，通常设为0禁用
    TransferSize_T SSize;           //源传输大小
    uint8_t DMode;
    TransferSize_T DSize;
    uint32_t MinorLoopTransBytes;   //单次DMA请求传输字节数--具体使用MLNO-MLOFFYES-MLOFFNO需进一步参考pg345
    uint32_t SLastAddrOffset;       //主循环结束后的源地址偏移量
    uint32_t DAddr;
    uint16_t DOffset;
//  bool EnLink;                    //默认不使用通道链接--因此寄存器为ELINKNO
    uint16_t MajorIterCount;        //当前主循环计数器，次循环结束时递减
    uint32_t DLastAddrAdjust;       //主循环结束后目标地址偏移量
//  uint8_t Bandwidth;              //用于缓冲，默认不使用
//  bool EnSG;                      //用于在主循环完成后，修改当前通道的TCD设置，默认不使用
    bool DisableReq;                //在主循环完成后，是否需要硬件自动清除通道请求，如果设为1，表示在主循环完成后，通道请求无效，需再次启动请求
    bool EnableHalfInt;             //主循环完成一半时产生中断
    bool EnableMajorInt;            //主循环完成时产生中断
    bool StartChan;                 //软件写1可发出通道请求，启动后硬件自动清零
    uint16_t StartMajorIterCount;   //无通道链接，使用ELINKNO寄存器，表示起始主循环次数
}DMA_TCD_Config_T;

void DMAMUX_Init_Yancy(uint8_t Channel,TrigSource_T TrigSource);
void DMA_TCD_Init (uint8_t Channel,DMA_TCD_Config_T TCD);
void DMA_StartChannel(uint8_t Channel);
void DMA_CancelTransfer(uint8_t Channel);
void DMA_ChannelReqEnable(uint8_t Channel);
void DMA_Init(void);

void DMA_CH0_IRQHandler();
void DMA_CH4_IRQHandler();
void DMA_CH3_IRQHandler();

void DMA_CH0_ADC0_Init(void);
void DMA_CH4_ADC1_Init(void);
void DMA_CH3_ADC1_V_Init(void);
#endif /* DMA_H_ */

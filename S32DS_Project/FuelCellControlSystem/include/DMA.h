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
    uint32_t SAddr;                 //婧愬湴鍧€--娉ㄦ剰涓�32浣嶅湴鍧€锛屽彲鑳介渶瑕佷娇鐢�&瀵诲潃
    uint16_t SOffset;               //婧愬湴鍧€鍋忕Щ閲�
    uint8_t SMode;                  //婧愭ā寮�--闇€瑕佽繘涓€姝ラ獙璇侊紝琛ㄧず浼犺緭2^娆℃柟娆★紝閫氬父璁句负0绂佺敤
    TransferSize_T SSize;           //婧愪紶杈撳ぇ灏�
    uint8_t DMode;
    TransferSize_T DSize;
    uint32_t MinorLoopTransBytes;   //鍗曟DMA璇锋眰浼犺緭瀛楄妭鏁�--鍏蜂綋浣跨敤MLNO-MLOFFYES-MLOFFNO闇€杩涗竴姝ュ弬鑰僷g345
    uint32_t SLastAddrOffset;       //涓诲惊鐜粨鏉熷悗鐨勬簮鍦板潃鍋忕Щ閲�
    uint32_t DAddr;
    uint16_t DOffset;
//  bool EnLink;                    //榛樿涓嶄娇鐢ㄩ€氶亾閾炬帴--鍥犳瀵勫瓨鍣ㄤ负ELINKNO
    uint16_t MajorIterCount;        //褰撳墠涓诲惊鐜鏁板櫒锛屾寰幆缁撴潫鏃堕€掑噺
    uint32_t DLastAddrAdjust;       //涓诲惊鐜粨鏉熷悗鐩爣鍦板潃鍋忕Щ閲�
//  uint8_t Bandwidth;              //鐢ㄤ簬缂撳啿锛岄粯璁や笉浣跨敤
//  bool EnSG;                      //鐢ㄤ簬鍦ㄤ富寰幆瀹屾垚鍚庯紝淇敼褰撳墠閫氶亾鐨凾CD璁剧疆锛岄粯璁や笉浣跨敤
    bool DisableReq;                //鍦ㄤ富寰幆瀹屾垚鍚庯紝鏄惁闇€瑕佺‖浠惰嚜鍔ㄦ竻闄ら€氶亾璇锋眰锛屽鏋滆涓�1锛岃〃绀哄湪涓诲惊鐜畬鎴愬悗锛岄€氶亾璇锋眰鏃犳晥锛岄渶鍐嶆鍚姩璇锋眰
    bool EnableHalfInt;             //涓诲惊鐜畬鎴愪竴鑸椂浜х敓涓柇
    bool EnableMajorInt;            //涓诲惊鐜畬鎴愭椂浜х敓涓柇
    bool StartChan;                 //杞欢鍐�1鍙彂鍑洪€氶亾璇锋眰锛屽惎鍔ㄥ悗纭欢鑷姩娓呴浂
    uint16_t StartMajorIterCount;   //鏃犻€氶亾閾炬帴锛屼娇鐢‥LINKNO瀵勫瓨鍣紝琛ㄧず璧峰涓诲惊鐜鏁�
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

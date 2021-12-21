#include "Cpu.h"
#include "S32K144.h"
#include "LPIT.h"

volatile uint32_t FreeRTOSRunTimeTicks=0;
void LPIT_Init(void)
{
    FreeRTOSRunTimeTicks=0;
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT0_CH0, &lpit1_ChnConfig0);

    INT_SYS_InstallHandler(LPIT0_Ch0_IRQn,LPIT0_IRQHandler,(isr_t*) 0);
    INT_SYS_SetPriority(LPIT0_Ch0_IRQn,LPIT0_CH0_INTPRI);
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);

    LPIT_DRV_StartTimerChannels(INST_LPIT1,1<<LPIT0_CH0);
}

void LPIT0_IRQHandler(void)
{
    FreeRTOSRunTimeTicks++;
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1,1<<LPIT0_CH0);
}

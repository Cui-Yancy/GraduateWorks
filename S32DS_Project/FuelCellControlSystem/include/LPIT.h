#ifndef LPIT_H_
#define LPIT_H_
#include <stdint.h>
#include "FreeRTOSConfig.h"

#define LPIT0_CH0           0
#define LPIT0_CH0_INTPRI    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+3)

extern volatile uint32_t FreeRTOSRunTimeTicks;

void LPIT_Init(void);
void LPIT0_IRQHandler(void);

#endif /* LPIT_H_ */

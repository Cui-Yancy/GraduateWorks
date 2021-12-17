#ifndef RTOS_H_
#define RTOS_H_

#include "FreeRTOSConfig.h"

#define CANRX_STACKSIZE     200
#define CANRX_PRIORITY      (configMAX_PRIORITIES-1)

#define CANRX_LENGTH        5
#define CANRX_ITEMSIZE      8

#define DEBUG_MODE          1

void HardwareInit();
void RTOS_Start();
void CANRX( void * pvParameters );

#endif /* RTOS_H_ */

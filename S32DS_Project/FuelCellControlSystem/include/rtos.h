#ifndef RTOS_H_
#define RTOS_H_

#include "FreeRTOSConfig.h"

//各任务配置，包括堆栈大小、优先级
#define CANRX_STACKSIZE     100
#define CANRX_PRIORITY      (configMAX_PRIORITIES-1)

#define CANTX_STACKSIZE     100
#define CANTX_PRIORITY      (configMAX_PRIORITIES-1)

#define SYSTEMSTATUS_STACKSIZE     400
#define SYSTEMSTATUS_PRIORITY      0

//队列配置
#define CANRX_LENGTH        5
#define CANTX_LENGTH        5


#define DEBUG_MODE          1

void HardwareInit();
void RTOS_Start();

void CANRX( void * pvParameters );
void CANTX( void * pvParameters );
void SYSTEMSTATUS( void * pvParameters );

#endif /* RTOS_H_ */

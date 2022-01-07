#ifndef RTOS_H_
#define RTOS_H_

#include "FreeRTOSConfig.h"

//各任务配置，包括堆栈大小、优先级
#define CANRX_STACKSIZE             100
#define CANRX_PRIORITY              (configMAX_PRIORITIES-1)

#define CANTX_STACKSIZE             100
#define CANTX_PRIORITY              (configMAX_PRIORITIES-1)

#define CONTROL_STACKSIZE           400
#define CONTROL_PRIORITY            (configMAX_PRIORITIES-2)

#define SENSOR_STACKSIZE            350
#define SENSOR_PRIORITY             (configMAX_PRIORITIES-3)

#define SYSTEMSTATUS_STACKSIZE      300
#define SYSTEMSTATUS_PRIORITY       0

//队列配置
#define CANRX_LENGTH                5
#define CANTX_LENGTH                5
#define PWM_LENGTH                  2
#define PAUSE_LENGTH                2

//事件标志组配置
typedef enum
{
    Event_T = 0,            //温度数组DMA传输完成标志位
    Event_I = 1,
    Event_V = 2,
    Event_On = 3,           //开机指令标志位
    Event_Off = 4,          //关机指令标志位
    Flag_Status = 5,        //系统状态标志位
}EventBits;
#define EVENT_BIT(n)                (1<<n)
#define SYSTEM_ON                   (1<<Flag_Status)
#define SYSTEM_OFF                  0

#define DEBUG_MODE                  0

void HardwareInit();
void RTOS_Start();

void CANRX( void * pvParameters );
void CANTX( void * pvParameters );
void SENSOR( void * pvParameters );
void CONTROL( void * pvParameters );
void SYSTEMSTATUS( void * pvParameters );

#endif /* RTOS_H_ */

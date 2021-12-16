#include "rtos.h"
#include "Cpu.h"
#include "CAN.h"
#include "gpio.h"
#include "stdio.h"

//#include "FreeRTOS.h"
//#include "task.h"
#include "rtos.h"

TaskHandle_t CANRX_Handler;

QueueHandle_t CANRX_Queue;

void HardwareInit()
{
	CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
						g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);
	GPIO_Init();
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
	CAN0_Init();
}

void RTOS_Start()
{
//    xTaskCreate((TaskFunction_t         ) CANRX             ,   //任务入口函数
//                (const char *           ) "CANRX"           ,   //任务名称
//                (configSTACK_DEPTH_TYPE ) CANRX_STACKSIZE   ,   //任务堆栈大小
//                (void *                 ) NULL              ,   //任务输入
//                (UBaseType_t            ) CANRX_PRIORITY    ,   //任务优先级
//                (TaskHandle_t *         ) &CANRX_Handler )  ;   //任务句柄
	CANRX_Queue = xQueueCreate(CANRX_LENGTH,CANRX_ITEMSIZE);
    if(CANRX_Queue == NULL){
      printf("Create CANRX_Queue Failed\r\n");
    }
    vTaskStartScheduler();
}

void CANRX( void * pvParameters )
{

}

#include "rtos.h"
#include "Cpu.h"
#include "CAN.h"
#include "gpio.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
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
#if DEBUG_MODE
    printf("UART Init finished\r\n");
#endif

	CAN0_Init();
#if DEBUG_MODE
    printf("CAN Init finished\r\n");
#endif
}

void RTOS_Start()
{
    BaseType_t retValue;
#if DEBUG_MODE
    printf("Welcome to FreeRTOS\r\n");
#endif
    retValue = xTaskCreate( (TaskFunction_t         ) CANRX             ,   //任务入口函数
                            (const char *           ) "CANRX"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) CANRX_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) CANRX_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &CANRX_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:CANRX task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("CANRX task create succeed\r\n");
        #endif
    }
    CANRX_Queue = xQueueCreate(CANRX_LENGTH,sizeof(CANMessage));
    if(!CANRX_Queue){
      printf("[%s,%d]:CANRX_Queue create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("CANRX queue create succeed\r\n");
        #endif
    }

#if DEBUG_MODE
    printf("Start OS Scheduler\r\n");
#endif
    vTaskStartScheduler();
}

void CANRX( void * pvParameters )
{
    BaseType_t retValue;
    CANMessage CAN_Message;
    while(1)
    {
        retValue = xQueueReceive(CANRX_Queue,&CAN_Message,portMAX_DELAY);
        PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_GREEN);
        if(retValue == pdTRUE)
        {
            printf("Receive CAN frame from CANRX queue succeed\r\n");
        }
        CAN_Message.MessageArry[7]++;
        CAN0_Send(CAN_Message);
        //vTaskDelay(1000);
    }
}

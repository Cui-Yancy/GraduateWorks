#include "rtos.h"
#include "Cpu.h"
#include "CAN.h"
#include "gpio.h"
#include "ADC0.h"
#include "ADC1.h"
#include "DMA.h"

#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "rtos.h"

TaskHandle_t CANRX_Handler;
TaskHandle_t CANTX_Handler;
TaskHandle_t SENSOR_Handler;
TaskHandle_t SYSTEMSTATUS_Handler;

QueueHandle_t CANRX_Queue;
QueueHandle_t CANTX_Queue;

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

    DMA_Init();

    ADC0_Init();
    ADC1_Init();
}

void RTOS_Start()
{
    BaseType_t retValue;
#if DEBUG_MODE
    printf("Welcome to FreeRTOS\r\n");
    printf("RemainHeapSize = %d\r\n",xPortGetFreeHeapSize());
    printf("MinRemainHeapSize = %d\r\n",xPortGetMinimumEverFreeHeapSize());
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

    retValue = xTaskCreate( (TaskFunction_t         ) CANTX             ,   //任务入口函数
                            (const char *           ) "CANTX"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) CANTX_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) CANTX_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &CANTX_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:CANTX task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("CANTX task create succeed\r\n");
        #endif
    }

    retValue = xTaskCreate( (TaskFunction_t         ) SENSOR             ,   //任务入口函数
                            (const char *           ) "SENSOR"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) SENSOR_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) SENSOR_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &SENSOR_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:CANTX task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("CANTX task create succeed\r\n");
        #endif
    }

    retValue = xTaskCreate( (TaskFunction_t         ) SYSTEMSTATUS             ,   //任务入口函数
                            (const char *           ) "SYSTEMSTATUS"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) SYSTEMSTATUS_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL                     ,   //任务输入
                            (UBaseType_t            ) SYSTEMSTATUS_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &SYSTEMSTATUS_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:SYSTEMSTATUS task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("SYSTEMSTATUS task create succeed\r\n");
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

    CANTX_Queue = xQueueCreate(CANTX_LENGTH,sizeof(CANMessage));
    if(!CANTX_Queue){
      printf("[%s,%d]:CANTX_Queue create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("CANTX queue create succeed\r\n");
        #endif
    }

#if DEBUG_MODE
    printf("Start OS Scheduler\r\n");
#endif
    vTaskStartScheduler();
}

//CAN接收任务
void CANRX( void * pvParameters )
{
    BaseType_t retValue;
    CANMessage CAN_Message;
    while(1)
    {
        retValue = xQueueReceive(CANRX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Receive CAN frame from CANRX queue succeed\r\n");
            #endif
            PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_BLUE);
        }
        else
        {
            printf("[%s,%d]:Receive CAN frame from CANRX queue failed\r\n",__FILE__,__LINE__);
        }

        CAN_Message.MessageArry[7]++;
        retValue = xQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Send CAN frame to CANTX queue succeed\r\n");
            #endif
            PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_BLUE);
        }
        else
        {
            printf("[%s,%d]:Send CAN frame to CANTX queue failed\r\n",__FILE__,__LINE__);
        }
        
        //vTaskDelay(1000);
    }
}

void CANTX( void * pvParameters )
{
    BaseType_t retValue;
    CANMessage CAN_Message;
    while(1)
    {
        retValue = xQueueReceive(CANTX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Receive CAN frame from CANTX queue succeed\r\n");
            #endif
        }
        else
        {
            printf("[%s,%d]:Receive CAN frame from CANTX queue failed\r\n",__FILE__,__LINE__);
        }

        CAN0_Send(CAN_Message);
    }
}

void SENSOR( void * pvParameters )
{
    double T = 0.000,I = 0.0,V = 0.00;
    CANMessage CAN_Message;
    BaseType_t retValue;
    while(1)
    {
        ADC0_CalculateT(&T,&CAN_Message);
        retValue = xQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Send CAN frame to CANTX queue succeed\r\n");
            #endif
        }
        else
        {
            printf("[%s,%d]:Send CAN frame to CANTX queue failed\r\n",__FILE__,__LINE__);
        }
        printf("T(°C) = %-14.4f",T);

        ADC1_CalculateV(&V,&CAN_Message);
        retValue = xQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Send CAN frame to CANTX queue succeed\r\n");
            #endif
        }
        else
        {
            printf("[%s,%d]:Send CAN frame to CANTX queue failed\r\n",__FILE__,__LINE__);
        }
            printf("V(V) = %-10.4f",V);

        ADC1_CalculateI(&I,&CAN_Message);
        retValue = xQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue == pdTRUE)
        {
            #if DEBUG_MODE
                printf("Send CAN frame to CANTX queue succeed\r\n");
            #endif
        }
        else
        {
            printf("[%s,%d]:Send CAN frame to CANTX queue failed\r\n",__FILE__,__LINE__);
        }
        printf("I(A) = %-10.4f\r\n",I);

        vTaskDelay(100);
    }
}

void SYSTEMSTATUS( void * pvParameters )
{
    /*查询系统任务状态*/
    UBaseType_t     ArraySize,x;
    TaskStatus_t    *StatusArray;           //任务信息结构体指针
    uint32_t        TotalRunTime;
    static uint32_t CallCount = 0;
    
    /*查询任务运行状态*/
    // eTaskState TaskState;
    // TaskState = eTaskGetState(CANRX_Handler);
    // printf("CANRX TaskState = %d\r\n",TaskState);

    //不推荐使用，会使用较大的堆栈空间，同时还会影响中断
    // vTaskList(TaskInfo);
    // printf("%s\r\n",TaskInfo);

    //UBaseType_t MinRemainSize;
    while(1)
    {
        /*查询任务高水位线，即运行过程中剩余最小堆栈*/
        // MinRemainSize = uxTaskGetStackHighWaterMark(SYSTEMSTATUS_Handler);
        // printf("MinRemainSize = %d\r\n",(uint16_t)MinRemainSize);

        CallCount++;
        printf("\r\ncall time count: %lu\r\n",CallCount);
        ArraySize = uxTaskGetNumberOfTasks();   //获取任务个数
        StatusArray = (TaskStatus_t *)pvPortMalloc(ArraySize * sizeof(TaskStatus_t));//为指针分配空间
        if(StatusArray != NULL){    //内存分配成功
            ArraySize = uxTaskGetSystemState((TaskStatus_t  *) StatusArray   ,  //存储任务状态信息的结构体数组
                                            (UBaseType_t    ) ArraySize     ,
                                            (uint32_t      *) &TotalRunTime );
            printf("TaskName   Pri Num Stack RunTime    Status\r\n");
            for(x=0;x<ArraySize;x++)
            {
                //通过串口打印出获取到的系统任务的有关信息，比如任务名称、
                //任务优先级和任务编号。
                printf("%-11s%-4d%-4d%-6d%-11lu%-6d\r\n",
                StatusArray[x].pcTaskName,
                (int)StatusArray[x].uxCurrentPriority,
                (int)StatusArray[x].xTaskNumber,
                StatusArray[x].usStackHighWaterMark,
                StatusArray[x].ulRunTimeCounter,
                StatusArray[x].eCurrentState);//任务编号表示创建先后顺序，首先创建开始任务
            }
        }
        else
        {
            printf("[%s,%d]:pvPortMalloc failed\r\n",__FILE__,__LINE__);
        }

        printf("RemainHeapSize = %d\r\n",xPortGetFreeHeapSize());
        printf("MinRemainHeapSize = %d\r\n",xPortGetMinimumEverFreeHeapSize());
        printf("\r\n");
        vPortFree(StatusArray);
        PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_GREEN);
        
        vTaskDelay(1000);
    }
}

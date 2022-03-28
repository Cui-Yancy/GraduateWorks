#include "rtos.h"
#include "Cpu.h"
#include "CAN.h"
#include "gpio.h"
#include "ADC0.h"
#include "ADC1.h"
#include "DMA.h"
#include "FTM.h"

#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "rtos.h"

TaskHandle_t CANRX_Handler;
TaskHandle_t CANTX_Handler;
TaskHandle_t SENSOR_Handler;
TaskHandle_t ON_Handler;
TaskHandle_t OFF_Handler;
TaskHandle_t RUN_Handler;
//TaskHandle_t CONTROL_Handler;
//TaskHandle_t TEST_Handler;
TaskHandle_t SYSTEMSTATUS_Handler;

QueueHandle_t CANRX_Queue;
QueueHandle_t CANTX_Queue;
QueueHandle_t PWM_Queue;
QueueHandle_t PAUSE_Queue;
QueueHandle_t TIV_Queue;

EventGroupHandle_t EventGroupHandler;

SemaphoreHandle_t UART_MutexSemaphore;

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

    FTM0_Init();        //FTM0_CH7用于产生PWM控制风扇转速
    FTM1_Init_Fixed();  //FTM1_CH0用于产生脉冲，开关排气阀门
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
        printf("[%s,%d]:SENSOR task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("SENSOR task create succeed\r\n");
        #endif
    }

    retValue = xTaskCreate( (TaskFunction_t         ) ON             ,   //任务入口函数
                            (const char *           ) "ON"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) ON_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) ON_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &ON_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:ON task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("ON task create succeed\r\n");
        #endif
    }

    retValue = xTaskCreate( (TaskFunction_t         ) OFF             ,   //任务入口函数
                            (const char *           ) "OFF"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) OFF_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) OFF_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &OFF_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:OFF task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("OFF task create succeed\r\n");
        #endif
    }

    retValue = xTaskCreate( (TaskFunction_t         ) RUN             ,   //任务入口函数
                            (const char *           ) "RUN"           ,   //任务名称
                            (configSTACK_DEPTH_TYPE ) RUN_STACKSIZE   ,   //任务堆栈大小
                            (void *                 ) NULL              ,   //任务输入
                            (UBaseType_t            ) RUN_PRIORITY    ,   //任务优先级
                            (TaskHandle_t *         ) &RUN_Handler )  ;   //任务句柄
    if(retValue != pdPASS)
    {
        printf("[%s,%d]:RUN task create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("RUN task create succeed\r\n");
        #endif
    }

    // retValue = xTaskCreate( (TaskFunction_t         ) CONTROL             ,   //任务入口函数
    //                         (const char *           ) "CONTROL"           ,   //任务名称
    //                         (configSTACK_DEPTH_TYPE ) CONTROL_STACKSIZE   ,   //任务堆栈大小
    //                         (void *                 ) NULL              ,   //任务输入
    //                         (UBaseType_t            ) CONTROL_PRIORITY    ,   //任务优先级
    //                         (TaskHandle_t *         ) &CONTROL_Handler )  ;   //任务句柄
    // if(retValue != pdPASS)
    // {
    //     printf("[%s,%d]:CONTROL task create failed\r\n",__FILE__,__LINE__);
    // }
    // else
    // {
    //     #if DEBUG_MODE
    //         printf("CONTROL task create succeed\r\n");
    //     #endif
    // }

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

    // retValue = xTaskCreate( (TaskFunction_t         ) TEST             ,   //任务入口函数
    //                         (const char *           ) "TEST"           ,   //任务名称
    //                         (configSTACK_DEPTH_TYPE ) TEST_STACKSIZE   ,   //任务堆栈大小
    //                         (void *                 ) NULL                     ,   //任务输入
    //                         (UBaseType_t            ) TEST_PRIORITY    ,   //任务优先级
    //                         (TaskHandle_t *         ) &TEST_Handler )  ;   //任务句柄
    // if(retValue != pdPASS)
    // {
    //     printf("[%s,%d]:TEST task create failed\r\n",__FILE__,__LINE__);
    // }
    // else
    // {
    //     #if DEBUG_MODE
    //         printf("TEST task create succeed\r\n");
    //     #endif
    // }

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

    PWM_Queue = xQueueCreate(PWM_LENGTH,sizeof(uint16_t));
    if(!PWM_Queue){
      printf("[%s,%d]:PWM Queue create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("PWM Queue create succeed\r\n");
        #endif
    }

    PAUSE_Queue = xQueueCreate(PAUSE_LENGTH,sizeof(uint32_t));
    if(!PAUSE_Queue){
      printf("[%s,%d]:PAUSE Queue create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("PAUSE Queue create succeed\r\n");
        #endif
    }

    TIV_Queue = xQueueCreate(TIV_LENGTH,sizeof(TIV_Type));
    if(!TIV_Queue){
      printf("[%s,%d]:TIV Queue create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        #if DEBUG_MODE
            printf("PAUSE Queue create succeed\r\n");
        #endif
    }

    UART_MutexSemaphore = xSemaphoreCreateMutex();   //互斥信号量在创建时会进行释放操作，xQueueGenericSend，因此初始化后有效
    if(UART_MutexSemaphore == NULL){                //创建不成功
        printf("UART MutexSemaphore create failed\r\n");
    }

    EventGroupHandler = xEventGroupCreate();
    if(EventGroupHandler == NULL){
        printf("[%s,%d]:EventGroup create failed\r\n",__FILE__,__LINE__);
    }
    else
    {
        xEventGroupClearBits(   EventGroupHandler,
                                EVENT_BIT(Event_On)|EVENT_BIT(Event_Off)|
                                EVENT_BIT(Flag_Status)|EVENT_BIT(Event_Ready));
        xEventGroupSetBits(EventGroupHandler,EVENT_BIT(AUTO));  //开机默认自动控制
        #if DEBUG_MODE
            printf("EventGroup create succeed\r\n");
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
    uint16_t PWMDuty = 0;
    uint32_t PAUSEDuration = 0;
    EventBits_t uxBits;
    while(1)
    {
        retValue = xQueueReceive(CANRX_Queue,&CAN_Message,portMAX_DELAY);
        if(retValue != pdTRUE)
        {
            printf("[%s,%d]:Receive CAN frame from CANRX queue failed\r\n",__FILE__,__LINE__);
        }
        else
        {
            #if DEBUG_MODE
                printf("Receive CAN frame from CANRX queue succeed\r\n");
            #endif
            PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_BLUE);

            switch (CAN_Message.ID)
            {
            case ID_PWMFAN_SPEED:   //接收PWM和脉冲，还应该与开关机状态相关联
                {
                    PWMDuty = (CAN_Message.MessageArry[0]+(((uint16_t)CAN_Message.MessageArry[1])<<8));
                    PWMDuty = PWMDuty>PWMFAN_MAX?PWMFAN_MAX:PWMDuty;
                    PWMDuty = PWMDuty<PWMFAN_MIN?PWMFAN_MIN:PWMDuty;
                    retValue = xQueueSend(PWM_Queue,&PWMDuty,5);
                    if(retValue != pdTRUE)
                    {
                        printf("[%s,%d]:Send PWM duty to PWM queue failed\r\n",__FILE__,__LINE__);
                    }
                    else
                    {
                        #if DEBUG_MODE
                            printf("Send CAN frame to CANTX queue succeed\r\n");
                        #endif
                    }
                }
                break;
            case ID_PAUSE_Dur:
                {
                    PAUSEDuration = ((uint32_t)CAN_Message.MessageArry[3] << 24) |
                                    ((uint32_t)CAN_Message.MessageArry[2] << 16) |
                                    ((uint32_t)CAN_Message.MessageArry[1] << 8)  |
                                    ((uint32_t)CAN_Message.MessageArry[0] << 0)  ;
                    retValue = xQueueSend(PAUSE_Queue,&PAUSEDuration,5);
                    if(retValue != pdTRUE)
                    {
                        printf("[%s,%d]:Send PAUSE duration to PAUSE queue failed\r\n",__FILE__,__LINE__);
                    }
                    else
                    {
                        #if DEBUG_MODE
                            printf("Send PAUSE duration to PAUSE queue succeed\r\n");
                        #endif
                    }
                }
                break;
            case ID_SYS_ON_OFF:
                {
                    if(CAN_Message.MessageArry[0] == 0) //关机请求
                    {
                        uxBits = xEventGroupGetBits(EventGroupHandler);
                        if((uxBits & EVENT_BIT(Flag_Status)) == SYSTEM_ON)
                        {
                            xEventGroupSetBits(EventGroupHandler,EVENT_BIT(Event_Off));
                        //不需要判断这个操作成功与否，可以判断是否执行了这个标志位事件
                        //因为可能会有任务在等待这个标志位，刚置高就消失了
                        // if((uxBits & EVENT_BIT(Event_Off)) != EVENT_BIT(Event_Off))
                        // {
                        //     printf("[%s,%d]:set event off bit failed\r\n",__FILE__,__LINE__);
                        // }
                        }
                        else
                        {
                            printf("Switch off failed, system already off\r\n");
                        }
                    }
                    else if(CAN_Message.MessageArry[0] == 1)
                    {
                        uxBits = xEventGroupGetBits(EventGroupHandler);
                        if((uxBits & EVENT_BIT(Flag_Status)) == SYSTEM_OFF)
                        {
                            xEventGroupSetBits(EventGroupHandler,EVENT_BIT(Event_On));
                        // if((uxBits & EVENT_BIT(Event_On)) != EVENT_BIT(Event_On))
                        // {
                        //     printf("[%s,%d]:set event on bit failed\r\n",__FILE__,__LINE__);
                        // }
                        }
                        else
                        {
                            printf("Switch on failed, system already on\r\n");
                        }
                    }
                }
                break;
            case ID_ControlMode:
                {
                    if(CAN_Message.MessageArry[0] == 0)
                    {
                        printf("switch to auto control\r\n");
                        xEventGroupSetBits(EventGroupHandler,EVENT_BIT(AUTO));
                    }
                    else if(CAN_Message.MessageArry[0] == 1)
                    {
                        printf("switch to manual control\r\n");
                        xEventGroupClearBits(EventGroupHandler,EVENT_BIT(AUTO));
                    }
                }
                break;
            default:
                break;
            }

            CAN_Message.MessageArry[7]++;
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
        }

        
        
        //vTaskDelay(1000);
    }
}

static void MyQueueSend(QueueHandle_t    xQueue,
                        const void       *pvItemToQueue,
                        TickType_t       xTicksToWait,
                        const char*      warning)
{
    BaseType_t retValue;
    retValue = xQueueSend(xQueue,pvItemToQueue,xTicksToWait);
    if(retValue == pdTRUE)
    {
        #if DEBUG_MODE
            printf("Send CAN frame to CANTX queue succeed\r\n");
        #endif
    }
    else
    {
        printf("Queue send failed log:%s\r\n",warning);
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
            CAN0_Send(CAN_Message);
        }
        else
        {
            printf("[%s,%d]:Receive CAN frame from CANTX queue failed\r\n",__FILE__,__LINE__);
        }

    }
}

void SENSOR( void * pvParameters )
{
    //double T = 0.000,I = 0.0,V = 0.00;
    TIV_Type TIV;
    CANMessage CAN_Message;
    //BaseType_t retValue;
    EventBits_t uxBits;
    while(1)
    {
        if(EventGroupHandler != NULL){
            uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
                                         (EventBits_t        ) EVENT_BIT(Event_T)|EVENT_BIT(Event_I)|EVENT_BIT(Event_V),  //等待的位
                                         (BaseType_t         ) pdTRUE,              //退出是否清零
                                         (BaseType_t         ) pdTRUE,             //是否等待所有位
                                         (TickType_t         ) portMAX_DELAY );     //阻塞时间
            if((uxBits & (EVENT_BIT(Event_T)|EVENT_BIT(Event_I)|EVENT_BIT(Event_V))) == (EVENT_BIT(Event_T)|EVENT_BIT(Event_I)|EVENT_BIT(Event_V)))
            {
                ADC0_CalculateT(&TIV.T,&CAN_Message);
                //printf("I=%.1f ",TIV.T);
                MyQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY,"1");

                ADC1_CalculateI(&TIV.I,&CAN_Message);
                //printf("I=%.1f ",TIV.I);
                MyQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY,"2");

                ADC1_CalculateV(&TIV.V,&CAN_Message);
                //printf("I=%.1f ",TIV.T);
                MyQueueSend(CANTX_Queue,&CAN_Message,portMAX_DELAY,"3");

                //MyQueueSend(TIV_Queue,&TIV,portMAX_DELAY,"4");
            }
            else
            {
                printf("wait TIV bits failed\r\n");
            }
        }
        else
        {
            printf("eventgroup invalid\r\n");
        }

        

        #if DEBUG_MODE
        if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
        {
            printf("T(°C) = %-14.4f",T);
            printf("V(V) = %-10.4f",V);
            printf("I(A) = %-10.4f\r\n",I);
            xSemaphoreGive(UART_MutexSemaphore);
        }
        #endif
        vTaskDelay(100);
    }
}

void ON( void * pvParameters )
{
    EventBits_t uxBits;
    while(1)
    {
        if(EventGroupHandler != NULL){
            uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
                                         (EventBits_t        ) EVENT_BIT(Event_On),  //等待的位
                                         (BaseType_t         ) pdTRUE,              //退出是否清零
                                         (BaseType_t         ) pdFALSE,             //是否等待所有位
                                         (TickType_t         ) portMAX_DELAY );                 //阻塞时间
            if((uxBits & EVENT_BIT(Event_On)) == EVENT_BIT(Event_On))
            {
                //开机流程：打开氢气阀门、排气阀门，吹扫5s，
                printf("Swich On\r\n");
                OpenH2();
                FTM1_CH0_GeneratePause_Fixed(0,BLOW_TIME*1000);
                FTM0_UpdatePWM(FTM0_PWMChannel,800);
                xEventGroupSetBits(EventGroupHandler,EVENT_BIT(Flag_Status));
                vTaskDelay(BLOW_TIME);
                //开机按下后，可以立马关闭，但是下一次打开系统需要等待5秒后才能再次打开
                xEventGroupSetBits(EventGroupHandler,EVENT_BIT(Event_Ready));
            }
            else
            {
                printf("wait ON bit failed\r\n");
            }
        }
        else
        {
            printf("event group invalid\r\n");
            vTaskDelay(1000);
        }
    }

}
void OFF( void * pvParameters )
{
    EventBits_t uxBits;
    while(1)
    {
        if(EventGroupHandler != NULL){
            uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
                                         (EventBits_t        ) EVENT_BIT(Event_Off),  //等待的位
                                         (BaseType_t         ) pdTRUE,              //退出是否清零
                                         (BaseType_t         ) pdFALSE,             //是否等待所有位
                                         (TickType_t         ) portMAX_DELAY );                 //阻塞时间
            if((uxBits & EVENT_BIT(Event_Off)) == EVENT_BIT(Event_Off))
            {
                printf("Swich Off\r\n");
                xEventGroupClearBits(EventGroupHandler,EVENT_BIT(Flag_Status));
                xEventGroupClearBits(EventGroupHandler,EVENT_BIT(Event_Ready));
                CloseH2();
                //等待温度下降***********************************
                FTM0_UpdatePWM(FTM0_PWMChannel,0);
            }
            else
            {
                printf("wait OFF bit failed\r\n");
            }
        }
        else
        {
            printf("event group invalid\r\n");
            vTaskDelay(1000);
        }
    }
}
void RUN( void * pvParameters )
{
    EventBits_t uxBits,value;
    BaseType_t retValue;
    uint16_t PWMDuty = 0;
    uint32_t PAUSEDuration = 0;
    while(1)
    {
        if(EventGroupHandler != NULL){
            uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
                                         (EventBits_t        ) EVENT_BIT(Event_Ready)|EVENT_BIT(Flag_Status),  //等待的位
                                         (BaseType_t         ) pdFALSE,              //退出是否清零
                                         (BaseType_t         ) pdTRUE,             //是否等待所有位
                                         (TickType_t         ) portMAX_DELAY );                 //阻塞时间
            if((uxBits & EVENT_BIT(Event_Ready)) == EVENT_BIT(Event_Ready))
            {
                value = xEventGroupGetBits(EventGroupHandler);
                value = (value>>AUTO) & 1;
                if(value)   //自动控制
                {
                    printf("auto control\r\n");
                    vTaskDelay(1000);
                }
                else        //手动控制
                {
                    retValue = xQueueReceive(PWM_Queue,&PWMDuty,10);
                    if(retValue == pdTRUE)
                    {
                        if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
                        {
                            printf("Receive PWMDuty from PWM_Queue succeed\r\nPWMDuty = %.2f%%\r\n",PWMDuty/100.0);
                            xSemaphoreGive(UART_MutexSemaphore);
                        }
                        FTM0_UpdatePWM(FTM0_PWMChannel,PWMDuty);
                    }

                    retValue = xQueueReceive(PAUSE_Queue,&PAUSEDuration,10);
                    if(retValue == pdTRUE)
                    {
                        if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
                        {
                            printf("Receive PAUSEDuration from PAUSE_Queue succeed\r\nPAUSEDuration = %.3fms\r\n",PAUSEDuration/1000.0);
                            xSemaphoreGive(UART_MutexSemaphore);
                        }
                        FTM1_CH0_GeneratePause_Fixed(0,PAUSEDuration);
                    }
                }
            }
            else
            {
                printf("wait Ready bit failed\r\n");
            }
        }
        else
        {
            printf("event group invalid\r\n");
            vTaskDelay(1000);
        }
    }
}

//需要添加上电后校准环节，先不添加负载，电流值为0
//最好是上电自动校准一次，然后可以上位机手动校准
// void CONTROL( void * pvParameters )
// {
//     BaseType_t retValue;
//     uint16_t PWMDuty = 0;
//     uint32_t PAUSEDuration = 0;
//     EventBits_t uxBits;
    
//     //vTaskDelay(2000);   //先等待2s，系统稳定后
//     while(1)
//     {
//         if(EventGroupHandler != NULL){
//             uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
//                                          (EventBits_t        ) EVENT_BIT(Event_On),  //等待的位
//                                          (BaseType_t         ) pdTRUE,              //退出是否清零
//                                          (BaseType_t         ) pdFALSE,             //是否等待所有位
//                                          (TickType_t         ) 1 );                 //阻塞时间
//             if((uxBits & EVENT_BIT(Event_On)) == EVENT_BIT(Event_On))
//             {
//                 //开机流程：打开氢气阀门、排气阀门，吹扫5s，
//                 //printf("Swich On\r\n");
//                 OpenH2();
//                 FTM1_CH0_GeneratePause_Fixed(0,5000*1000);
//                 FTM0_UpdatePWM(FTM0_PWMChannel,800);
//                 xEventGroupSetBits(EventGroupHandler,EVENT_BIT(Flag_Status));
//             }
//             uxBits = xEventGroupWaitBits((EventGroupHandle_t ) EventGroupHandler,   //时间标志句柄
//                                          (EventBits_t        ) EVENT_BIT(Event_Off),  //等待的位
//                                          (BaseType_t         ) pdTRUE,              //退出是否清零
//                                          (BaseType_t         ) pdFALSE,             //是否等待所有位
//                                          (TickType_t         ) 2 );                 //阻塞时间
//             if((uxBits & EVENT_BIT(Event_Off)) == EVENT_BIT(Event_Off))
//             {
//                 //printf("Swich Off\r\n");
//                 CloseH2();
//                 FTM0_UpdatePWM(FTM0_PWMChannel,0);
//                 xEventGroupClearBits(EventGroupHandler,EVENT_BIT(Flag_Status));
//             }
//         }

//         retValue = xQueueReceive(PWM_Queue,&PWMDuty,1);
//         if(retValue == pdTRUE)
//         {
//             if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
//             {
//                 printf("Receive PWMDuty from PWM_Queue succeed\r\nPWMDuty = %.2f%%\r\n",PWMDuty/100.0);
//                 xSemaphoreGive(UART_MutexSemaphore);
//             }
//             FTM0_UpdatePWM(FTM0_PWMChannel,PWMDuty);
//         }

//         retValue = xQueueReceive(PAUSE_Queue,&PAUSEDuration,1);
//         if(retValue == pdTRUE)
//         {
//             if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
//             {
//                 printf("Receive PAUSEDuration from PAUSE_Queue succeed\r\nPAUSEDuration = %.3fms\r\n",PAUSEDuration/1000.0);
//                 xSemaphoreGive(UART_MutexSemaphore);
//             }
//             FTM1_CH0_GeneratePause_Fixed(1000,PAUSEDuration);
//         }
//     }
// }

// void TEST( void * pvParameters )
// {
//     // uint16_t Data[ADC0_ResultNum];
//     // uint32_t Sum=0;
//     // uint16_t Average=0,min=4095,max=0;
//     // int i=0;
//     while(1)
//     {
//          vTaskDelay(5000);
//         // //memcpy(Data,ADC0_Result,ADC0_ResultNum*2);
//         // Sum=0;
//         // Average=0;
//         // min=4095;
//         // max=0;
//         // for(i=0;i<ADC0_ResultNum;i++)
//         // {
//         //     Data[i]=ADC0_Result[i];
//         //     Sum+=Data[i];
//         //     if(Data[i]>max) max=Data[i];
//         //     if(Data[i]<min) min=Data[i];
//         // }
//         // Average = Sum/ADC0_ResultNum;
//         // printf("\r\noriginal:\r\n");
//         // printf("min = %d\tmax = %d\taverage = %d\r\n",min,max,Average);

//         // uint8_t * Num =(uint8_t *) pvPortMalloc((max-min+1)*sizeof(uint8_t));
//         // for(i=0;i<=(max-min);i++)
//         //     Num[i]=0;
//         // for(i=0;i<ADC0_ResultNum;i++)
//         // {
//         //     Num[Data[i]-min]++;
//         // }
//         // printf("Data-min freq\r\n");
//         // for(i=0;i<=(max-min);i++)
//         //     printf("%-9d%.3f\r\n",i,(float)Num[i]/ADC0_ResultNum);

//         // vPortFree(Num);
        
//     }
// }

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
            if(xSemaphoreTake(UART_MutexSemaphore,10) == pdTRUE)
            {
                printf("TaskName   Pri Num Stack RunTime\r\n");
                for(x=0;x<ArraySize;x++)
                {
                    //通过串口打印出获取到的系统任务的有关信息，比如任务名称、
                    //任务优先级和任务编号。
                    printf("%-11s%-4d%-4d%-6d%lu\r\n",
                    StatusArray[x].pcTaskName,
                    (int)StatusArray[x].uxCurrentPriority,
                    (int)StatusArray[x].xTaskNumber,
                    StatusArray[x].usStackHighWaterMark,
                    StatusArray[x].ulRunTimeCounter);//任务编号表示创建先后顺序，首先创建开始任务
                }
                printf("RemainHeapSize = %d\r\n",xPortGetFreeHeapSize());
                printf("MinRemainHeapSize = %d\r\n",xPortGetMinimumEverFreeHeapSize());
                printf("\r\n");
                xSemaphoreGive(UART_MutexSemaphore);
            }
        }
        else
        {
            printf("[%s,%d]:pvPortMalloc failed\r\n",__FILE__,__LINE__);
        }

        vPortFree(StatusArray);
        PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_GREEN);
        
        vTaskDelay(5000);
    }
}

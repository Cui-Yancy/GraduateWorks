#ifndef RTOS_H_
#define RTOS_H_

#include "FreeRTOSConfig.h"

//各任务配置，包括堆栈大小、优先级
#define CANRX_STACKSIZE             400
#define CANRX_PRIORITY              (configMAX_PRIORITIES-1)

#define CANTX_STACKSIZE             100
#define CANTX_PRIORITY              (configMAX_PRIORITIES-3)

// #define CONTROL_STACKSIZE           400
// #define CONTROL_PRIORITY            (configMAX_PRIORITIES-2)

#define ON_STACKSIZE                200
#define ON_PRIORITY                 (configMAX_PRIORITIES-2)

#define OFF_STACKSIZE               200
#define OFF_PRIORITY                (configMAX_PRIORITIES-2)

#define RUN_STACKSIZE               450
#define RUN_PRIORITY                (configMAX_PRIORITIES-2)

#define SENSOR_STACKSIZE            350
#define SENSOR_PRIORITY             (configMAX_PRIORITIES-4)

#define SYSTEMSTATUS_STACKSIZE      300
#define SYSTEMSTATUS_PRIORITY       0

// #define TEST_STACKSIZE              500
// #define TEST_PRIORITY               1

//队列配置
#define CANRX_LENGTH                5
#define CANTX_LENGTH                5
#define PWM_LENGTH                  2
#define PAUSE_LENGTH                1   //原来是2***
#define TIV_LENGTH                  1
#define PWMREAL_LENGTH              1
#define PID_LENGTH                  1

#define BLOW_TIME                   5000    //开机吹扫5000ms
#define TIVUPDATE_TIME              80     //更新TIV时间间隔 80ms
#define CONTROL_TIME                100     //PID调节控制间隔时间，必须比TIV时间大，否则没更新状态
#define STARTPWMDUTY                800     //开机起始PWM占空比 8%
#define EXHAUSTINTERVAL_TIME        10000   //排气间隔时间
#define PAUSE_TIME                  100     //排气脉冲时间

#define Kp_INIT                     20//12//20
#define Ki_INIT                     0.001//0.545//0.001
#define Kd_INIT                     0.1//66//0.1

typedef struct
{
    double T;
    double I;
    double V;
}TIV_Type;

//事件标志组配置
typedef enum
{
    Event_T = 0,            //温度数组DMA传输完成标志位
    Event_I = 1,
    Event_V = 2,
    Event_On = 3,           //开机指令标志位
    Event_Off = 4,          //关机指令标志位
    Flag_Status = 5,        //系统状态标志位，用于表示开关机状态，1为已开机
    AUTO = 6,               //手动/自动标志位
    NeedBlow = 7,           //系统是否需要开机吹扫，开机操作后需要吹扫
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
//void CONTROL( void * pvParameters );
void ON( void * pvParameters );
void OFF( void * pvParameters );
void RUN( void * pvParameters );
//void TEST( void * pvParameters );
void SYSTEMSTATUS( void * pvParameters );

#endif /* RTOS_H_ */

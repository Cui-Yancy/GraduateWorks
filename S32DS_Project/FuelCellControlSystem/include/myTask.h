#ifndef MYTASK_H_
#define MYTASK_H_

#include "FreeRTOS.h"
#include "task.h"

//任务1
#define Task1_StackSize					300
#define Task1_Priority					2
TaskHandle_t Task1_Handler;
void Task1( void * pvParameters );

//任务2
#define Task2_StackSize					300
#define Task2_Priority					3
TaskHandle_t Task2_Handler;
void Task2( void * pvParameters );

#endif /* MYTASK_H_ */

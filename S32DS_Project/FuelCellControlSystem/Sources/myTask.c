#include "myTask.h"
#include <stdio.h>
#include "pin_mux.h"

void Task1( void * pvParameters )
{
	uint8_t Task1_Num=0;
	TickType_t previousWakeTickCount = xTaskGetTickCount();
	while(1){
		Task1_Num++;
		if(Task1_Num==5){
			printf("Delete Task2!\r\n");
			vTaskDelete(Task2_Handler);
		}else if(Task1_Num==255){
			Task1_Num = 5;
		}
		PINS_DRV_TogglePins(PTD,1<<0);
		printf("Task1 is running! %d\r\n",Task1_Num);
		xTaskDelayUntil(&previousWakeTickCount,1000);
	}
}

void Task2( void * pvParameters )
{
	uint8_t Task2_Num=0;
	TickType_t previousWakeTickCount = xTaskGetTickCount();
	while(1){
		Task2_Num++;
		printf("Task2 is running! %d\r\n",Task2_Num);
		xTaskDelayUntil(&previousWakeTickCount,1000);
	}
}

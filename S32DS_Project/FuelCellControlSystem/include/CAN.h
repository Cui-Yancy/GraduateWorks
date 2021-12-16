#ifndef CAN_H_
#define CAN_H_

#include "stdbool.h"
#include "flexcan_driver.h"

typedef struct
{
    bool isExtendFrame;
    uint32_t ID;
    uint8_t MessageArry[8];
}CANMessage;

typedef enum
{
    ID_T = 0x11111U,
    ID_V = 0x22222U,
    ID_I = 0x33333U,
    ID_PWNFAN_DUTY_PID = 0x44444U,
}CANID_Send;

typedef enum
{
    ID_PWMFAN_SPEED     =   0x7777777U,     //用于手动控制风扇转速
    ID_SYS_ON_OFF       =   0x1234567U,
    ID_ControlMode      =   0x666666U,      //用于表示系统控制模式，包括：风扇占空比手动/自动
    ID_PID_Parm         =   0x999999U,      //用于更新PID参数
}CANID_Receive;

#define TX_MessageBuffer	16U		//必须大于等于8，与FIFO个数有关，详细参照Pg.1597
#define CAN0INTPRIORITY     6

void CAN0_Init(void);
void CAN0_Receive(CANMessage *message);
void CAN0_IRQHandler();
void CAN0_Send(CANMessage message);

#endif /* CAN_H_ */

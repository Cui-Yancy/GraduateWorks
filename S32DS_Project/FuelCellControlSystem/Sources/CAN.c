#include "S32K144.h"
#include "CAN.h"
#include "flexcan_driver.h"
#include "canCom1.h"

static void CAN_Send(CAN_Type *Base,uint8_t TX_MB,CANMessage message);

uint32_t IdFilter[]={0x7777777,0x1234567,0x666666,0x999999};
static flexcan_id_table_t ID_Table={
    .isRemoteFrame = false,
    .isExtendedFrame = true,
    .idFilter = IdFilter
};

/* 使能BUF5中断，即接收中断
 * 将接收队列缓冲区设置为接收扩展帧，ID限制在列表内
 * 使能CAN中断
 */
void CAN0_Init(void){
	FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
	CAN0->IFLAG1=0xFFFFFFFF;
	CAN0->IMASK1 = (1<<CAN_IFLAG1_BUF5I_SHIFT)/*|(1<<TX_MessageBuffer)*/;		//
	FLEXCAN_DRV_ConfigRxFifo(INST_CANCOM1,FLEXCAN_RX_FIFO_ID_FORMAT_A,&ID_Table);
	//FLEXCAN_DRV_SetRxFifoGlobalMask(INST_CANCOM1,FLEXCAN_MSG_ID_STD,0);		//
	FLEXCAN_DRV_SetRxFifoGlobalMask(INST_CANCOM1,FLEXCAN_MSG_ID_EXT,0xFFFFFFFF);//

	INT_SYS_InstallHandler(CAN0_ORed_0_15_MB_IRQn,CAN0_IRQHandler,(isr_t*) 0);
	INT_SYS_SetPriority(CAN0_ORed_0_15_MB_IRQn,CAN0INTPRIORITY);
	INT_SYS_EnableIRQ(CAN0_ORed_0_15_MB_IRQn);
}

//接收时，参考50.4.6 Rx FIFO structure pg.1645
void CAN0_Receive(CANMessage *message)
{
    uint32_t RxCODE,RxIDHIT;
    message->isExtendFrame = (CAN0->RAMn[0] >> 21) & 1;
    RxCODE= (CAN0->RAMn[0] & 0x00300000) >> 20;				//
    message->ID = (CAN0->RAMn[1] & CAN_WMBn_ID_ID_MASK)  >> (message->isExtendFrame ? 0 : 18);		//
    message->MessageArry[0] = (CAN0->RAMn[2] >> 24) & 0xFF ;
    message->MessageArry[1] = (CAN0->RAMn[2] >> 16) & 0xFF ;
    message->MessageArry[2] = (CAN0->RAMn[2] >> 8 ) & 0xFF ;
    message->MessageArry[3] = (CAN0->RAMn[2] >> 0 ) & 0xFF ;
    message->MessageArry[4] = (CAN0->RAMn[3] >> 24) & 0xFF ;
    message->MessageArry[5] = (CAN0->RAMn[3] >> 16) & 0xFF ;
    message->MessageArry[6] = (CAN0->RAMn[3] >> 8 ) & 0xFF ;
    message->MessageArry[7] = (CAN0->RAMn[3] >> 0 ) & 0xFF ;
    RxIDHIT   =  CAN0->RXFIR;									//
    (void)RxCODE;
    (void)RxIDHIT;
}

//发送时，参考50.4.3 Message buffer structure pg.1635
static void CAN_Send(CAN_Type *Base,uint8_t TX_MB,CANMessage message){
	uint32_t TimeOut = 200;
	if(((Base->IFLAG1) >> TX_MB) & 1){
		Base->IFLAG1 = (1<<TX_MB);
	}
	while((((Base->RAMn[ TX_MB*4 + 0]) >> 24) & 0xF) != 0x8){
		TimeOut--;
		if(TimeOut==0)
			break;
	}
	Base->RAMn[TX_MB*4+1]=message.ID<<(message.isExtendFrame ? 0 : 18);
	Base->RAMn[TX_MB*4+2]=(message.MessageArry[0]<<24)|(message.MessageArry[1]<<16)|
						  (message.MessageArry[2]<<8)|message.MessageArry[3];
	Base->RAMn[TX_MB*4+3]=(message.MessageArry[4]<<24)|(message.MessageArry[5]<<16)|
						  (message.MessageArry[6]<<8)|message.MessageArry[7];
	Base->RAMn[TX_MB*4+0]=0x0C400000 | 8 <<CAN_WMBn_CS_DLC_SHIFT| (message.isExtendFrame<<CAN_WMBn_CS_IDE_SHIFT);
}

void CAN0_Send(CANMessage message)
{
	CAN_Send(CAN0,TX_MessageBuffer,message);
}

void CAN0_IRQHandler(){
	if((CAN0->IFLAG1>>CAN_IFLAG1_BUF5I_SHIFT) & 1){
		CANMessage CAN_Message;
		CAN0_Receive(&CAN_Message);
		CAN0_Send(CAN_Message);
		CAN0->IFLAG1 = 1<<CAN_IFLAG1_BUF5I_SHIFT;//
	}else{
		CAN0->IFLAG1 = ~((uint16_t)((1<<CAN_IFLAG1_BUF5I_SHIFT)));
	}
}

#include "can_interface.h"

CANContext can1_context;
CANContext can2_context;

/*----------------------------------------------------------------------------------------
函数名： void CAN_Init(void)
功  能：初始化can类相关变量
----------------------------------------------------------------------------------------*/
void CANContext::CAN_Init(CAN_TypeDef * hCANx,uint8_t ide,uint8_t rtr, uint8_t dlc){
    CANx = hCANx;
    CANx_TxMsg.IDE = ide;
    CANx_TxMsg.RTR = rtr;
    CANx_TxMsg.DLC = dlc;
}

/*----------------------------------------------------------------------------------------
函数名： void CanSendMessage(void)
功  能：can发送
----------------------------------------------------------------------------------------*/
uint8_t CANContext::CanSendMessage(void){
    return CAN_Transmit(CANx,&CANx_TxMsg);
}
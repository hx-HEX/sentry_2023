#pragma once
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

class CANContext
{
public:
    CAN_TypeDef * CANx;
    CanRxMsg CANx_RxMsg;
    CanTxMsg CANx_TxMsg;

    void CAN_Init(CAN_TypeDef * hCANx,uint8_t ide,uint8_t rtr, uint8_t dlc);
    void CanSendMessage(void);

    CANContext() {};
    ~CANContext() {};
};

extern CANContext can1_context;
extern CANContext can2_context;
#pragma once

#include "usart_interface.h"

#define DJI_RC_RECEIVE_SIZE            ((uint8_t)18)

#define RC_CH_VALUE_MIN             ((uint16_t)364)
#define RC_CH_VALUE_OFFSET          ((uint16_t)1024)
#define RC_CH_VALUE_MAX             ((uint16_t)1684)
#define RC_CH_VALUE_DEAD            ((uint16_t)50)
#define RC_CH_VALUE_RANGE           ((uint16_t)660)

#define RC_SW_UP                    ((uint8_t)1)
#define RC_SW_MID                   ((uint8_t)3)
#define RC_SW_DOWN                  ((uint8_t)2)

class DJIRC {
private:
    struct RCChannel {
        uint16_t Ch0;
        uint16_t Ch1;
        uint16_t Ch2;
        uint16_t Ch3;
        uint16_t Ch4;
        uint8_t SW_L;
        uint8_t SW_R;
    };
    
public:
    uint8_t m_data_receive_frame[DJI_RC_RECEIVE_SIZE];
    RCChannel channel;
    Usart* usart_djirc;

    DJIRC(){
        usart_djirc = new Usart();
    }

    void SetUartHandle(USART_TypeDef* USARTx);
};


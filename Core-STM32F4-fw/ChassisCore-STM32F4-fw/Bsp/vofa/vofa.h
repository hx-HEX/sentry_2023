#pragma once

#include "usart_interface.h"

#define VOFA_DATA_SEND_FLOAT_NUM ( 21 )
#define VOFA_DATA_RECEIVE_SIZE ( 50 )
#define VOFA_DATA_SEND_SIZE    ((uint8_t)4 + 4 * VOFA_DATA_SEND_FLOAT_NUM)

class Vofa {
public:
    struct VofaDataSendFrame {
        float m_data[VOFA_DATA_SEND_FLOAT_NUM];  
        unsigned char m_tail[4];
    };
	
	VofaDataSendFrame m_data_send_frame;
    Usart* usart_vofa;

    Vofa() {
        for (int i = 0; i < VOFA_DATA_SEND_FLOAT_NUM; i++) {
		    m_data_send_frame.m_data[i] = 0;
        }
		m_data_send_frame.m_tail[0] = 0x00;
        m_data_send_frame.m_tail[1] = 0x00;
        m_data_send_frame.m_tail[2] = 0x80;
        m_data_send_frame.m_tail[3] = 0x7F;

        usart_vofa = new Usart();
	};

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
#pragma once

#include "usart_interface.h"

#define CHASSIS_CORE_DATA_SEND_FLOAT_NUM          ((uint8_t)7)
#define CHASSIS_CORE_DATA_SEND_SHORT_NUM          ((uint8_t)2)
#define CHASSIS_CORE_DATA_SEND_CHAR_NUM          ((uint8_t)3)
#define CHASSIS_CORE_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)2)
#define CHASSIS_CORE_DATA_RECEIVE_SHORT_NUM      ((uint8_t)0)
#define CHASSIS_CORE_DATA_RECEIVE_CHAR_NUM       ((uint8_t)0)
#define CHASSIS_CORE_DATA_SEND_SIZE            ((uint8_t)4 + 4 * CHASSIS_CORE_DATA_SEND_FLOAT_NUM + \
                                        2 * CHASSIS_CORE_DATA_SEND_SHORT_NUM + CHASSIS_CORE_DATA_SEND_CHAR_NUM)
#define CHASSIS_CORE_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * CHASSIS_CORE_DATA_RECEIVE_FLOAT_NUM + \
                                            CHASSIS_CORE_DATA_RECEIVE_CHAR_NUM)

#define CHASSIS_CORE_RECEIVE_HEAD_SIZE  3


class Chassis_Core {
public:
    #pragma pack(1)
    struct Chassis_CoreDataSendFrame {
        unsigned char m_head[2];
        int16_t m_sdata[CHASSIS_CORE_DATA_SEND_SHORT_NUM];
        float m_fdata[CHASSIS_CORE_DATA_SEND_FLOAT_NUM];
        unsigned char m_cdata[CHASSIS_CORE_DATA_SEND_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    struct Chassis_CoreDataReceiveFrame {
        unsigned char m_head[2];
        int16_t m_sdata[CHASSIS_CORE_DATA_RECEIVE_SHORT_NUM];
        float m_fdata[CHASSIS_CORE_DATA_RECEIVE_FLOAT_NUM];
        unsigned char m_cdata[CHASSIS_CORE_DATA_RECEIVE_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    #pragma pack()

	unsigned char m_data_send_size;
	Chassis_CoreDataSendFrame m_data_send_frame;
	Chassis_CoreDataReceiveFrame m_data_receive_frame;
    Usart* usart_chassis_core;

    Chassis_Core() {
		m_data_send_size = CHASSIS_CORE_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;

        m_data_receive_frame.m_head[0]  = 0x55;
        m_data_receive_frame.m_head[1]  = 0x00;
        m_data_receive_frame.m_tail[0]  = 0x00;
        m_data_receive_frame.m_tail[1]  = 0xAA;

        usart_chassis_core = new Usart();
	};

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
#pragma once

#include "usart_interface.h"

#define RIGHT_GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)3)
#define RIGHT_GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)0)
#define RIGHT_GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)5)
#define RIGHT_GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)7)
#define RIGHT_GIMBAL_DATA_RECEIVE_SHORT_NUM       ((uint8_t)0)
#define RIGHT_GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)2)
#define RIGHT_GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * RIGHT_GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * RIGHT_GIMBAL_DATA_SEND_SHORT_NUM + RIGHT_GIMBAL_DATA_SEND_CHAR_NUM)
#define RIGHT_GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * RIGHT_GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                            RIGHT_GIMBAL_DATA_RECEIVE_CHAR_NUM)


class Right_Gimbal {
public:
    #pragma pack(1)
    struct Right_GimbalDataSendFrame {
        unsigned char m_head[2];
        int16_t m_sdata[RIGHT_GIMBAL_DATA_SEND_SHORT_NUM]; 
        float m_fdata[RIGHT_GIMBAL_DATA_SEND_FLOAT_NUM];
        unsigned char m_cdata[RIGHT_GIMBAL_DATA_SEND_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    struct Right_GimbalDataReceiveFrame {
        unsigned char m_head[2];
        int16_t m_sdata[RIGHT_GIMBAL_DATA_RECEIVE_SHORT_NUM];
        float m_fdata[RIGHT_GIMBAL_DATA_RECEIVE_FLOAT_NUM];
        unsigned char m_cdata[RIGHT_GIMBAL_DATA_RECEIVE_CHAR_NUM];
        unsigned char m_tail[2];
    };
    #pragma pack()

	unsigned char m_data_send_size;
	Right_GimbalDataSendFrame m_data_send_frame;
	Right_GimbalDataReceiveFrame m_data_receive_frame;
    Usart* usart_right_gimbal;

    Right_Gimbal() {
		m_data_send_size = RIGHT_GIMBAL_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;

        m_data_receive_frame.m_head[0]  = 0x55;
        m_data_receive_frame.m_head[1]  = 0x00;
        m_data_receive_frame.m_tail[0]  = 0x00;
        m_data_receive_frame.m_tail[1]  = 0xAA;

        usart_right_gimbal = new Usart();
	};

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
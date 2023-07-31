#pragma once

#include "usart_interface.h"

#define YAW_GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)21)
#define YAW_GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)0)
#define YAW_GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)1)
#define YAW_GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)1)//yaw_current
#define YAW_GIMBAL_DATA_RECEIVE_SHORT_NUM       ((uint8_t)2)
#define YAW_GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)0)
#define YAW_GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * YAW_GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * YAW_GIMBAL_DATA_SEND_SHORT_NUM + YAW_GIMBAL_DATA_SEND_CHAR_NUM)
#define YAW_GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * YAW_GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                            YAW_GIMBAL_DATA_RECEIVE_CHAR_NUM)


class Yaw_Gimbal {
public:
    #pragma pack(1)
    struct Yaw_GimbalDataSendFrame {
        unsigned char m_head[2];
        int16_t m_sdata[YAW_GIMBAL_DATA_SEND_SHORT_NUM];
        float m_fdata[YAW_GIMBAL_DATA_SEND_FLOAT_NUM];
        unsigned char m_cdata[YAW_GIMBAL_DATA_SEND_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    struct Yaw_GimbalDataReceiveFrame {
        unsigned char m_head[2];
        int16_t m_sdata[YAW_GIMBAL_DATA_RECEIVE_SHORT_NUM];
        float m_fdata[YAW_GIMBAL_DATA_RECEIVE_FLOAT_NUM];
        unsigned char m_cdata[YAW_GIMBAL_DATA_RECEIVE_CHAR_NUM];
        unsigned char m_tail[2];
    };
    #pragma pack()

	unsigned char m_data_send_size;
	Yaw_GimbalDataSendFrame m_data_send_frame;
	Yaw_GimbalDataReceiveFrame m_data_receive_frame;
    Usart* usart_yaw_gimbal;

    Yaw_Gimbal() {
		m_data_send_size = YAW_GIMBAL_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;

        m_data_receive_frame.m_head[0]  = 0x55;
        m_data_receive_frame.m_head[1]  = 0x00;
        m_data_receive_frame.m_tail[0]  = 0x00;
        m_data_receive_frame.m_tail[1]  = 0xAA;

        usart_yaw_gimbal = new Usart();
	};

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
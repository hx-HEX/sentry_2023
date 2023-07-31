#pragma once 

#include "usart_interface.h"

#define GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)1)
#define GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)2)//left_yaw_output,right_yaw_output
#define GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)0)
#define GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)21)//主控vofa和主控数据
#define GIMBAL_DATA_RECEIVE_SHORT_NUM       ((uint8_t)0)
#define GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)1)//mode
#define GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * GIMBAL_DATA_SEND_SHORT_NUM + GIMBAL_DATA_SEND_CHAR_NUM)
#define GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                        2 * GIMBAL_DATA_RECEIVE_SHORT_NUM + GIMBAL_DATA_RECEIVE_CHAR_NUM)


class Gimbal {
public:
    #pragma pack(1)
    struct GimbalDataSendFrame {
        unsigned char m_head[2];
        int16_t m_sdata[GIMBAL_DATA_SEND_SHORT_NUM];
        float m_fdata[GIMBAL_DATA_SEND_FLOAT_NUM];
        unsigned char m_cdata[GIMBAL_DATA_SEND_CHAR_NUM];
        unsigned char m_tail[2];
    };
    struct GimbalDataReceiveFrame {
        unsigned char m_head[2];
        float m_fdata[GIMBAL_DATA_RECEIVE_FLOAT_NUM]; 
        unsigned char m_cdata[GIMBAL_DATA_RECEIVE_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    #pragma pack()

    unsigned char m_data_send_size = GIMBAL_DATA_SEND_SIZE;
    GimbalDataSendFrame m_data_send_frame;
    GimbalDataReceiveFrame m_data_receive_frame;
    Usart* usart_gimbal;

    Gimbal(){
        m_data_send_size = GIMBAL_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;
        m_data_send_frame.m_tail[0]  = 0x00;
        m_data_send_frame.m_tail[1]  = 0xAA;

        m_data_receive_frame.m_head[0]  = 0x55;
        m_data_receive_frame.m_head[1]  = 0x00;
        m_data_receive_frame.m_tail[0]  = 0x00;
        m_data_receive_frame.m_tail[1]  = 0xAA;

        usart_gimbal = new Usart();
    };

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
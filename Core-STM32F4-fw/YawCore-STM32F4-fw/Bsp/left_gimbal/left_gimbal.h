#pragma once

#include "usart_interface.h"

#define LEFT_GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)3)//pitch,yaw目标值，弹速
#define LEFT_GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)0)
#define LEFT_GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)5)//shoot、run、balance_infantry_num、enemy_color,m_scan_flag
#define LEFT_GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)7)//辅瞄5个数，pitch_fb,yaw_fb
#define LEFT_GIMBAL_DATA_RECEIVE_SHORT_NUM       ((uint8_t)0)
#define LEFT_GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)2)//shoot_flag,enemy_number
#define LEFT_GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * LEFT_GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * LEFT_GIMBAL_DATA_SEND_SHORT_NUM + LEFT_GIMBAL_DATA_SEND_CHAR_NUM)
#define LEFT_GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * LEFT_GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                            LEFT_GIMBAL_DATA_RECEIVE_CHAR_NUM)


class Left_Gimbal {
public:
    #pragma pack(1)
    struct Left_GimbalDataSendFrame {
        unsigned char m_head[2];
        int16_t m_sdata[LEFT_GIMBAL_DATA_SEND_SHORT_NUM]; 
        float m_fdata[LEFT_GIMBAL_DATA_SEND_FLOAT_NUM];
        unsigned char m_cdata[LEFT_GIMBAL_DATA_SEND_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    struct Left_GimbalDataReceiveFrame {
        unsigned char m_head[2];
        int16_t m_sdata[LEFT_GIMBAL_DATA_RECEIVE_SHORT_NUM];
        float m_fdata[LEFT_GIMBAL_DATA_RECEIVE_FLOAT_NUM];
        unsigned char m_cdata[LEFT_GIMBAL_DATA_RECEIVE_CHAR_NUM];
        unsigned char m_tail[2];
    };
    #pragma pack()

	unsigned char m_data_send_size;
	Left_GimbalDataSendFrame m_data_send_frame;
	Left_GimbalDataReceiveFrame m_data_receive_frame;
    Usart* usart_left_gimbal;

    Left_Gimbal() {
		m_data_send_size = LEFT_GIMBAL_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;

        m_data_receive_frame.m_head[0]  = 0x55;
        m_data_receive_frame.m_head[1]  = 0x00;
        m_data_receive_frame.m_tail[0]  = 0x00;
        m_data_receive_frame.m_tail[1]  = 0xAA;

        usart_left_gimbal = new Usart();
	};

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
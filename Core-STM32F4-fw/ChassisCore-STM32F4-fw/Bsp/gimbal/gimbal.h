#pragma once

#include "usart_interface.h"

#define GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)2)//yaw_angle,chassis_world_speed
#define GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)0)
#define GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)0)
#define GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)7)//vx,vy,w,radar_yaw,navigation_x,navigation_y,yaw_des
#define GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)3)//chassis_mode,yaw_gimbal_mode,calibrate_flag
#define GIMBAL_DATA_RECEIVE_SHORT_NUM      ((uint8_t)2)//leftyaw_output,rightyaw_output
#define GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * GIMBAL_DATA_SEND_SHORT_NUM + GIMBAL_DATA_SEND_CHAR_NUM)
#define GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                            2 * GIMBAL_DATA_RECEIVE_SHORT_NUM + GIMBAL_DATA_RECEIVE_CHAR_NUM)
#define GIMBAL_SEND_BUFF_SIZE      2000
#define GIMBAL_SEND_HEAD_SIZE      3
                                  


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
        int16_t m_sdata[GIMBAL_DATA_RECEIVE_SHORT_NUM];
        float m_fdata[GIMBAL_DATA_RECEIVE_FLOAT_NUM];
        unsigned char m_cdata[GIMBAL_DATA_RECEIVE_CHAR_NUM];
        unsigned char m_tail[2];
    };
    #pragma pack()

    unsigned char gimbal_send_buff[GIMBAL_SEND_BUFF_SIZE];
    unsigned int gimbal_send_buff_cnt;
	unsigned char m_data_send_size;
	GimbalDataSendFrame m_data_send_frame;
	GimbalDataReceiveFrame m_data_receive_frame;
    Usart* usart_gimbal;

    Gimbal() {
		m_data_send_size = GIMBAL_DATA_SEND_SIZE;

        m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;
        m_data_send_frame.m_tail[0] = 0x00;
        m_data_send_frame.m_tail[1] = 0xAA;

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
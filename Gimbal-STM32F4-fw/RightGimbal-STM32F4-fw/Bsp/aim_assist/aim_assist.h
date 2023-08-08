#pragma once

#include "usart_interface.h"

#define AIM_ASSIST_DATA_SEND_FLOAT_NUM          ((uint8_t)10)//pitch_fb,yaw_fb,bullet_speed,balance
#define AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)9)//pitch_des,yaw_des,supply_bullet_flag,change_direction_flag,其它五个数
#define AIM_ASSIST_DATA_SEND_SIZE               ((uint8_t)6 + 4 * AIM_ASSIST_DATA_SEND_FLOAT_NUM)
#define AIM_ASSIST_DATA_RECEIVE_SIZE            ((uint8_t)6 + 4 * AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM)


class AimAssist {
public:
    #pragma pack(1)
    struct AimAssistDataSendFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data[AIM_ASSIST_DATA_SEND_FLOAT_NUM];
        unsigned char m_tail[2];
    };
    struct AimAssistDataReceiveFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data_f[AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM];  
        unsigned char m_tail[2];
    };
    #pragma pack()
	
	unsigned char m_data_send_size;
	AimAssistDataSendFrame m_data_send_frame;
    AimAssistDataReceiveFrame m_data_receive_frame;

    Usart* usart_assist;

    AimAssist(void){
		m_data_send_size = AIM_ASSIST_DATA_SEND_SIZE;
		
		m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;
		m_data_send_frame.m_id = 0x00;
		m_data_send_frame.m_length = AIM_ASSIST_DATA_SEND_FLOAT_NUM;
        for (int i = 0; i < AIM_ASSIST_DATA_SEND_FLOAT_NUM; i++) {
            m_data_send_frame.m_data[i] = 0;
        }
		m_data_send_frame.m_tail[0] = 0x00;
        m_data_send_frame.m_tail[1] = 0xAA;

		m_data_receive_frame.m_head[0] = 0x55;
        m_data_receive_frame.m_head[1] = 0x00;
		m_data_receive_frame.m_id = 0x00;
		m_data_receive_frame.m_length = AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM;
        for (int i = 0; i < AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM; i++) {
            m_data_receive_frame.m_data_f[i] = 0;
        }
		m_data_receive_frame.m_tail[0] = 0x00;
        m_data_receive_frame.m_tail[1] = 0xAA;

        usart_assist = new Usart();
    };

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
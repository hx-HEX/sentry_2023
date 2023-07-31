#pragma once

#include "usart_interface.h"
#include "referee.h"

#define NAVIGATION_DATA_SEND_FLOAT_NUM          ((uint8_t)4+5)
#define NAVIGATION_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)9)
#define NAVIGATION_DATA_RECEIVE_CHAR_NUM        ((uint8_t)1+2)
#define NAVIGATION_DATA_SEND_SIZE               ((uint8_t)6 + 4 * NAVIGATION_DATA_SEND_FLOAT_NUM)
#define NAVIGATION_DATA_RECEIVE_SIZE            ((uint8_t)6 + 4 * NAVIGATION_DATA_RECEIVE_FLOAT_NUM + NAVIGATION_DATA_RECEIVE_CHAR_NUM)
#define NAVIGATION_SEND_BUFF_SIZE      2000
#define NAVIGATION_SEND_HEAD_SIZE 3


class Navigation {
public:
    #pragma pack(1)
    struct NavigationDataSendFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data[NAVIGATION_DATA_SEND_FLOAT_NUM];
        unsigned char m_tail[2];
    };
    struct NavigationDataReceiveFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data_f[NAVIGATION_DATA_RECEIVE_FLOAT_NUM];  
        unsigned char m_data_c[NAVIGATION_DATA_RECEIVE_CHAR_NUM];  
        unsigned char m_tail[2];
    };
    #pragma pack()
	
	unsigned char m_data_send_size;
	NavigationDataSendFrame m_data_send_frame;
    NavigationDataReceiveFrame m_data_receive_frame;
    unsigned char navigation_send_buff[NAVIGATION_SEND_BUFF_SIZE];
    unsigned int navigation_send_buff_cnt;

    Usart* usart_navigation;

    float m_arrival_goal_x;
    float m_arrival_goal_y;

    Navigation(void){
        m_arrival_goal_x = 3.11;
        m_arrival_goal_y = -0.73;
		
		m_data_send_size = NAVIGATION_DATA_SEND_SIZE;
		
		m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;
		m_data_send_frame.m_id = 0x00;
		m_data_send_frame.m_length = NAVIGATION_DATA_SEND_FLOAT_NUM;
        for (int i = 0; i < NAVIGATION_DATA_SEND_FLOAT_NUM; i++) {
            m_data_send_frame.m_data[i] = 0;
        }
		m_data_send_frame.m_tail[0] = 0x00;
        m_data_send_frame.m_tail[1] = 0xAA;

		m_data_receive_frame.m_head[0] = 0x55;
        m_data_receive_frame.m_head[1] = 0x00;
		m_data_receive_frame.m_id = 0x00;
		m_data_receive_frame.m_length = NAVIGATION_DATA_RECEIVE_FLOAT_NUM;
        for (int i = 0; i < NAVIGATION_DATA_RECEIVE_FLOAT_NUM; i++) {
            m_data_receive_frame.m_data_f[i] = 0;
        }
        for (int i = 0; i < NAVIGATION_DATA_RECEIVE_CHAR_NUM; i++) {
            m_data_receive_frame.m_data_c[i] = 0;
        }
		m_data_receive_frame.m_tail[0] = 0x00;
        m_data_receive_frame.m_tail[1] = 0xAA;

        usart_navigation = new Usart();

        navigation_send_buff_cnt = 0;
    };

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};
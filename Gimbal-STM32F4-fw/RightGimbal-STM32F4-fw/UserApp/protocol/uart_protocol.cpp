#include "uart_protocol.h"
#include "user_global.h"
#include "verify.h"

void UART_Decode(EventBits_t EventValue){
    static EventBits_t usart_vofa_bit;
    static EventBits_t usart_gimbal_bit;
    static EventBits_t usart_aimassist_bit;

    usart_vofa_bit = EventValue & 0x04;
    usart_gimbal_bit = EventValue & 0x08;
    usart_aimassist_bit = EventValue & 0x20;
	
    if(usart_vofa_bit){
		G_system_monitor.Vofa_Receive_cnt++;
    }

    if(usart_gimbal_bit){
		UART_GimbalDataDecode(G_gimbal.usart_gimbal->USART_RT.pMailbox);
        G_system_monitor.Gimbal_Receive_cnt++;
    }

    if(usart_aimassist_bit){
		UART_AimAssitDataDecode(G_aim_assist.usart_assist->USART_RT.pMailbox);
        G_system_monitor.Aimassist_Receive_cnt++;
    }
}

void UART_GimbalDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,GIMBAL_DATA_RECEIVE_SIZE)) {
		memcpy(&(G_gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_gimbal.m_data_receive_frame));	
    }
}

void UART_AimAssitDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,AIM_ASSIST_DATA_RECEIVE_SIZE)) {
		memcpy(&(G_aim_assist.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_aim_assist.m_data_receive_frame));	
    }
}

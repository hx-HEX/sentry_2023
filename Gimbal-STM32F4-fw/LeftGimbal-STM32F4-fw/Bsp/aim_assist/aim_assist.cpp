#include "aim_assist.h"

/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给AimAssist模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void AimAssist::SetUartHandle(USART_TypeDef* USARTx){
    usart_assist->USART_RT.USARTx = USARTx;
    usart_assist->USART_Init((uint8_t*)&m_data_receive_frame,AIM_ASSIST_DATA_RECEIVE_SIZE,(uint8_t*)&m_data_send_frame,AIM_ASSIST_DATA_SEND_SIZE,0x20);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void AimAssist::SendData(void){
    usart_assist->USART_Transmit();
}
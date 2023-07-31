#include "referee.h"

/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给Referee模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void Referee::SetUartHandle(USART_TypeDef* USARTx){
    usart_referee->USART_RT.USARTx = USARTx;
    usart_referee->USART_Init((uint8_t*)&m_data_receive_buff,REFEREE_RECEIVE_SIZE,(uint8_t*)&m_data_send_buff,REFEREE_SEND_SIZE,0x02);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void Referee::SendData(void){
    usart_referee->USART_Transmit();
}
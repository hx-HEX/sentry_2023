#include "navigation.h"

/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给Navigation模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void Navigation::SetUartHandle(USART_TypeDef* USARTx){
    usart_navigation->USART_RT.USARTx = USARTx;
    usart_navigation->USART_Init((uint8_t*)&m_data_receive_frame,NAVIGATION_DATA_RECEIVE_SIZE,(uint8_t*)&navigation_send_buff,m_data_send_size,0x10);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void Navigation::SendData(void){
    usart_navigation->USART_Transmit();
}

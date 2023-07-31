#include "djirc.h"

/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给djirc模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void DJIRC::SetUartHandle(USART_TypeDef* USARTx){
    usart_djirc->USART_RT.USARTx = USARTx;
    usart_djirc->USART_Init((uint8_t*)&m_data_receive_frame,DJI_RC_RECEIVE_SIZE,0,0,0x01);
}
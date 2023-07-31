#include "vofa.h"


/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给vofa模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void Vofa::SetUartHandle(USART_TypeDef* USARTx){
    usart_vofa->USART_RT.USARTx = USARTx;
    usart_vofa->USART_Init(0,0,(uint8_t*)&m_data_send_frame,VOFA_DATA_SEND_SIZE,0x04);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void Vofa::SendData(void){
    usart_vofa->USART_Transmit();
}
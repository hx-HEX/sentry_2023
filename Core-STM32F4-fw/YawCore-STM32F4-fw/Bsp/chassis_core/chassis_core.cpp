#include "chassis_core.h"

/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给Chassis_Core模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void Chassis_Core::SetUartHandle(USART_TypeDef* USARTx){
    usart_chassis_core->USART_RT.USARTx = USARTx;
    usart_chassis_core->USART_Init((uint8_t*)&m_data_receive_frame,CHASSIS_CORE_DATA_RECEIVE_SIZE,(uint8_t*)&m_data_send_frame,CHASSIS_CORE_DATA_SEND_SIZE,0x02);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void Chassis_Core::SendData(void){
    usart_chassis_core->USART_Transmit();
}
#include "yaw_gimbal.h"
/*----------------------------------------------------------------------------------------
函数名：void SetUartHandle(USART_TypeDef* USARTx)
功  能：给Yaw_Gimbal模块设置串口，并初始化串口类
----------------------------------------------------------------------------------------*/
void Yaw_Gimbal::SetUartHandle(USART_TypeDef* USARTx){
    usart_yaw_gimbal->USART_RT.USARTx = USARTx;
    usart_yaw_gimbal->USART_Init((uint8_t*)&m_data_receive_frame,YAW_GIMBAL_DATA_RECEIVE_SIZE,(uint8_t*)&m_data_send_frame,YAW_GIMBAL_DATA_SEND_SIZE,0x04);
}

/*----------------------------------------------------------------------------------------
函数名：void SendData(void)
功  能：发送函数
----------------------------------------------------------------------------------------*/
void Yaw_Gimbal::SendData(void){
    usart_yaw_gimbal->USART_Transmit();
}
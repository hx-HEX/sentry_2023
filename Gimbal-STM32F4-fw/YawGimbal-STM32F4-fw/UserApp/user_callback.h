#ifndef USER_CALLBACK
#define USER_CALLBACK

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

void User_SysTickCallback(void);
void User_UART_RX_Callback(USART_TypeDef* USARTx);
void User_UART_Fps_Update(USART_TypeDef* USARTx);
void User_CAN_RX_Callback(CAN_TypeDef * hCANx);

#ifdef __cplusplus
}
#endif

#endif
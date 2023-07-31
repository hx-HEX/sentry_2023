#pragma once 

#include "FreeRTOS.h"
#include "event_groups.h"
#include "usart_interface.h"

void UART_Decode(EventBits_t EventValue);
void UART_GimbalDataDecode(unsigned char* buffer);
void UART_RSDataDecode(u8* buffer, u16 Size);
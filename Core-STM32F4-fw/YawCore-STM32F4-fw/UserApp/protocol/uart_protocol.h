#pragma once 

#include "FreeRTOS.h"
#include "event_groups.h"
#include "usart_interface.h"
#include "djirc.h"

void UART_Decode(EventBits_t EventValue);
void UART_DJIRCDataDecode(void);
void UART_NavigationDataDecode(unsigned char* buffer);
void UART_ChassisCoreDataDecode(unsigned char* buffer,uint16_t size);
void UART_YawGimbalDataDecode(unsigned char* buffer);
void UART_LeftGimbalDataDecode(unsigned char* buffer);
void UART_RightGimbalDataDecode(unsigned char* buffer);
void UART_RSDataDecode(u8* buffer, u16 Size);
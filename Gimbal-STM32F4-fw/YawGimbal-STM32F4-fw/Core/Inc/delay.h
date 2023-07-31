#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>
#include "stm32f4xx.h"
#include "misc.h"

#define SYSTEM_SUPPORT_OS		1
//如果使用OS,则包括下面的头文件即可
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void delay_ms(uint32_t t);
void delay_us(uint32_t t);

#ifdef __cplusplus
}
#endif


#endif

#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "stm32f4xx.h"
#include "misc.h"

#define SYSTEM_SUPPORT_OS		1
//如果使用OS,则包括下面的头文件即可
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#endif

void SysTick_Configuration(void);
#endif 


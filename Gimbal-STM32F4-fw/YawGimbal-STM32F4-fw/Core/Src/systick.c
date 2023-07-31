#include "systick.h"

//SYSTICK的时钟固定为AHB时钟，基础例程里面SYSTICK时钟频率为AHB/8
//这里为了兼容FreeRTOS，所以将SYSTICK的时钟频率改为AHB的频率！,即168M
//SYSCLK:系统时钟频率
void SysTick_Configuration(void)
{
	u8 SYSCLK=168;
	u32 reload;
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
	reload=SYSCLK;							//每秒钟的计数次数 单位为M	   
	reload*=1000000/configTICK_RATE_HZ;		//根据configTICK_RATE_HZ设定溢出时间  即168000
											//reload为24位寄存器,最大值:16777216,在168M下,约合0.0998s左右	
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/configTICK_RATE_HZ断一次	 即168M/168000=1khz
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK     
}

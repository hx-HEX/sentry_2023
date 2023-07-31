#include "bsp_init.h"

void bsp_init(void)
{
    //配置中断分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //系统时钟配置
		SysTick_Configuration();
    //GPIO配置
    GPIO_Configure();
    //PWM输出
    TIM4_Configuration();
    //参考时钟配置
    TIM5_Configuration();
    //串口配置
    USART2_Configuration();
    USART3_Configuration();
    USART6_Configuration();
    delay_ms(100);
    //CAN配置
    CAN1_Configuration();
    CAN2_Configuration();
    SPI_Configuration();
    delay_ms(100);
}

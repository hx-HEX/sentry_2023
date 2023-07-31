#include "tim.h"


/****************************************************************************************************
函数名称: TIM4_Configuration()
函数功能: PWM输出
输入参数: 无
返回参数: 无
备   注:  
****************************************************************************************************/
uint32_t servoInitAngle = 0;
void TIM4_Configuration(void)
{
	GPIO_InitTypeDef         	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         	TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//168MHz

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

    TIM_TimeBaseStructure.TIM_Prescaler     = 840-1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period        = 2000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;          //????,cnt<ccr???
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = servoInitAngle*200/180+50;   //???
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;      //?????
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);   //TIM1 PWM???????
    TIM_Cmd(TIM4, ENABLE);
}



/****************************************************************************************************
函数名称: TIM5_Configuration()
函数功能: 操作系统定时器时间参考
输入参数: 无
返回参数: 无
备   注:  
****************************************************************************************************/
//TIM5 提供时间参考
#define TIM5_Prescaler  84-1
#define TIM5_Period     4294967296-1        //4294967296us=71min，时间够用了，不做多余处理
//定时器初始化，定时器5是32位通用定时器，APB1=84MHz，1us进一次中断,只有定时器2和定时器5是32位，其他定时器为16位，不能用作时间参考
void TIM5_Configuration(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
	TIM_TimeBaseStructure.TIM_Prescaler         = TIM5_Prescaler;
	TIM_TimeBaseStructure.TIM_Period            = TIM5_Period;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    TIM_Cmd(TIM5, ENABLE);
}

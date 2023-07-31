#include "delay.h"

/*************************************************************************
函 数 名：delay_us
函数功能：延时t us
*************************************************************************/
void delay_us(uint32_t time)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt = 0;
    uint32_t reload = SysTick -> LOAD;

    ticks = time*168;       //延时指定时间需要的节拍数
    told = SysTick -> VAL;  //刚进入时的计数器值

    while(1){
        tnow = SysTick -> VAL;
        if(tnow != told){
            //SYSTICK是一个递减的计数器
            if(tnow < told) tcnt += told - tnow;    //说明计数器没有递减到重装载
            else    tcnt += reload - tnow + told;   //说明计数器重装载了
            told = tnow;
            if(tcnt >= ticks)   break;              //计数值到达了指定的节拍数
        }
    }
}

/*************************************************************************
函 数 名：delay_ms
函数功能：延时t ms
*************************************************************************/
void delay_ms(uint32_t time)
{
    uint32_t i;
    for(i=0; i<time; i++) delay_us(1000);
}

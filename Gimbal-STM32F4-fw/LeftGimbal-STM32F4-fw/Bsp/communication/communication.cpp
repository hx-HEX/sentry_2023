#include "communication.h"
#include "user_global.h"
#include "can_interface.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_can.h"

void InitCommunication(void){
    //串口类配置
    G_vofa.SetUartHandle(USART3);
    G_gimbal.SetUartHandle(USART2);
    G_aim_assist.SetUartHandle(USART6);
    //CAN类配置
    can1_context.CAN_Init(CAN1,CAN_ID_STD, CAN_RTR_DATA, 0X08);
    can2_context.CAN_Init(CAN2,CAN_ID_STD, CAN_RTR_DATA, 0X08);
}

void StartCommunication(void){
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE); //开启空闲中断
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE); //开启空闲中断
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE); //开启空闲中断

    /*CAN中断使能寄存器（CAN_IER）*/
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//接收中断:FIFO 0消息挂起中断
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//接收中断:FIFO 0消息挂起中断

}
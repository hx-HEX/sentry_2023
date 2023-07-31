#include "user_callback.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "usart_interface.h"
#include "user_global.h"
#include "can_interface.h"
#include "can_protocol.h"

uint16_t Clear_IT;
extern EventGroupHandle_t USARTHandler;

#define DETECT_MONITOR(name) G_system_monitor.name##_fps = \
G_system_monitor.name##_cnt; G_system_monitor.name##_cnt = 0

#define DETECT_RS_MONITOR(name) G_referee_monitor.name##_fps = \
G_referee_monitor.name##_cnt; G_referee_monitor.name##_cnt = 0

extern "C"{
    extern void xPortSysTickHandler(void);
}

void User_SysTickCallback(void){
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){
        xPortSysTickHandler();	
    }

    if(G_system_monitor.SysTickTime % 1000 == 0) {
		// Main Task Monitor
		DETECT_MONITOR(DataVisualizeTask);
		DETECT_MONITOR(RobotControlTask);
		DETECT_MONITOR(LedTask);
		DETECT_MONITOR(CoreDataSendTask);
		DETECT_MONITOR(AimAssistDataSendTask);
		DETECT_MONITOR(Vofa_Receive);
		DETECT_MONITOR(Gimbal_Receive);
		DETECT_MONITOR(Aimassist_Receive);


		// IT Monitor
		DETECT_MONITOR(UART1_rx);
		DETECT_MONITOR(UART2_rx);
		DETECT_MONITOR(UART3_rx);
		DETECT_MONITOR(UART4_rx);
		DETECT_MONITOR(UART5_rx);
		DETECT_MONITOR(UART6_rx);
		DETECT_MONITOR(CAN1_rx);
		DETECT_MONITOR(CAN2_rx);
	}

	G_system_monitor.SysTickTime++;
}


void User_UART_RX_Callback(USART_TypeDef* USARTx){
	Usart* Usartx;
	BaseType_t Result,xHigherPriorityTaskWoken;

	if (G_vofa.usart_vofa->USART_RT.USARTx == USARTx){
		Usartx = G_vofa.usart_vofa;
	}
	else if (G_gimbal.usart_gimbal->USART_RT.USARTx == USARTx){
		Usartx = G_gimbal.usart_gimbal;
	}
	else if(G_aim_assist.usart_assist->USART_RT.USARTx == USARTx)
	{
		Usartx = G_aim_assist.usart_assist;
	}

	User_UART_Fps_Update(USARTx);

	if(USART_GetITStatus(Usartx->USART_RT.USARTx, USART_IT_IDLE)!= RESET){
		Clear_IT = Usartx->USART_RT.USARTx->SR;
		Clear_IT = Usartx->USART_RT.USARTx->DR;//先读SR后读DR清楚中断标志位

		if(Usartx->USART_Receive() == Usartx->USART_RT.MbLen){
			if(USARTHandler != NULL){
				Result = xEventGroupSetBitsFromISR(USARTHandler,Usartx->USART_RT.Usart_event_bit,&xHigherPriorityTaskWoken);
				if(Result != pdFALSE){
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		}
	}	
}


void User_UART_Fps_Update(USART_TypeDef* USARTx){
	if(USARTx == USART1)	G_system_monitor.UART1_rx_cnt++;
	else if(USARTx == USART2)	G_system_monitor.UART2_rx_cnt++;
	else if(USARTx == USART3)	G_system_monitor.UART3_rx_cnt++;
	else if(USARTx == UART4)	G_system_monitor.UART4_rx_cnt++;
	else if(USARTx == UART5)	G_system_monitor.UART5_rx_cnt++;
	else if(USARTx == USART6)	G_system_monitor.UART6_rx_cnt++;
}


void User_CAN_RX_Callback(CAN_TypeDef * hCANx){
	CANContext* canx_context;
	BaseType_t Result,xHigherPriorityTaskWoken;

	if(hCANx == CAN1)	canx_context = &can1_context;
	else if(hCANx == CAN2)	canx_context = &can2_context;

	if(CAN_GetITStatus(hCANx,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(hCANx, CAN_IT_FMP0);
		CAN_Receive(hCANx, CAN_FIFO0, &canx_context->CANx_RxMsg);

		CAN_Decode(canx_context);

		if(hCANx == CAN1)	G_system_monitor.CAN1_rx_cnt++;
		else if(hCANx == CAN2)	G_system_monitor.CAN2_rx_cnt++;
	}
}
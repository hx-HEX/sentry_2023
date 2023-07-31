#include "usart_interface.h"

uint8_t UA1RxDMAbuf[USART1_RXDMA_LEN] = {0};
uint8_t UA2RxDMAbuf[USART2_RXDMA_LEN] = {0};
uint8_t UA3RxDMAbuf[USART3_RXDMA_LEN] = {0};
uint8_t UA4RxDMAbuf[UART4_RXDMA_LEN] = {0};
uint8_t UA5RxDMAbuf[UART5_RXDMA_LEN] = {0};
uint8_t UA6RxDMAbuf[USART6_RXDMA_LEN] = {0};

/*----------------------------------------------------------------------------------------
函数名： void USART_Init(void)
功  能：初始化串口类相关变量
----------------------------------------------------------------------------------------*/
void Usart::USART_Init(uint8_t* pMailbox,uint16_t MbLen,uint8_t* pTxbuf,uint16_t txlen,uint8_t Usart_event_bit){
	if(USART_RT.USARTx == USART1){
		USART_RT.DMAr_Streamx = DMA2_Stream2;
		USART_RT.pDMAbuf = UA1RxDMAbuf;
		USART_RT.DMALen = USART1_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA2_Stream7;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF7;
	}

	else if(USART_RT.USARTx == USART2){
		USART_RT.DMAr_Streamx = DMA1_Stream5;
		USART_RT.pDMAbuf = UA2RxDMAbuf;
		USART_RT.DMALen = USART2_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA1_Stream6;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF6;
	}

	else if(USART_RT.USARTx == USART3){
		USART_RT.DMAr_Streamx = DMA1_Stream1;
		USART_RT.pDMAbuf = UA3RxDMAbuf;
		USART_RT.DMALen = USART3_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA1_Stream3;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF3;
	}

	else if(USART_RT.USARTx == UART4){
		USART_RT.DMAr_Streamx = DMA1_Stream2;
		USART_RT.pDMAbuf = UA4RxDMAbuf;
		USART_RT.DMALen = UART4_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA1_Stream4;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF4;
	}

	else if(USART_RT.USARTx == UART5){
		USART_RT.DMAr_Streamx = DMA1_Stream0;
		USART_RT.pDMAbuf = UA5RxDMAbuf;
		USART_RT.DMALen = UART5_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA1_Stream7;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF7;
	}

	else if(USART_RT.USARTx == USART6){
		USART_RT.DMAr_Streamx = DMA2_Stream1;
		USART_RT.pDMAbuf = UA6RxDMAbuf;
		USART_RT.DMALen = USART6_RXDMA_LEN;
		USART_RT.rxSize = 0;

		USART_RT.DMAt_Streamx = DMA2_Stream6;
		USART_RT.DMA_IT_TCIFx = DMA_IT_TCIF6;
	}

	USART_RT.pMailbox = pMailbox;
	USART_RT.MbLen = MbLen;
	USART_RT.pTxbuf = pTxbuf;
	USART_RT.txlen = txlen;
	USART_RT.Usart_event_bit = Usart_event_bit;
}

/*----------------------------------------------------------------------------------------
函数名：USHORT16 USART_Receive(USART_RX_TypeDef* USARTx)
功  能：计算一次空闲中断接受数据长度，对DMA缓冲区数据一定长度数据进行存储
----------------------------------------------------------------------------------------*/
uint16_t Usart::USART_Receive(void)
{ 
	USART_RT.rxConter = USART_RT.DMALen - DMA_GetCurrDataCounter(USART_RT.DMAr_Streamx);  //本次DMA缓冲区填充到的位置

	USART_RT.rxBufferPtr += USART_RT.rxSize;  //上次DMA缓冲区填充到的位置

	if(USART_RT.rxBufferPtr >= USART_RT.DMALen)//说明DMA缓冲区已经满了一次
	{
		USART_RT.rxBufferPtr %= USART_RT.DMALen;
	}

	if(USART_RT.rxBufferPtr < USART_RT.rxConter) //上次和这次在同一段缓冲区时
	{
		USART_RT.rxSize = USART_RT.rxConter - USART_RT.rxBufferPtr; //计算本次接收数据的长度
		if(USART_RT.rxSize <= USART_RT.MbLen) //接收的数据长度不超过期望数据长度，把数据写进邮箱，防止数组越界
		{
			memcpy(USART_RT.pMailbox,USART_RT.pDMAbuf + USART_RT.rxBufferPtr,USART_RT.rxSize);//将接收的数据复制到邮箱数组里
		}
	}
	else //上次和这次在不同段的缓冲区时
	{
		USART_RT.rxSize = USART_RT.rxConter + USART_RT.DMALen - USART_RT.rxBufferPtr;//计算本次接收数据的长度
		if(USART_RT.rxSize <= USART_RT.MbLen) //接收的数据长度不超过期望数据长度，把数据写进邮箱，防止数组越界
		{
			memcpy(USART_RT.pMailbox,USART_RT.pDMAbuf + USART_RT.rxBufferPtr,USART_RT.rxSize-USART_RT.rxConter);//将接收的数据复制到邮箱数组里
			memcpy(USART_RT.pMailbox + USART_RT.rxSize-USART_RT.rxConter,USART_RT.pDMAbuf,USART_RT.rxConter);//将接收的数据复制到邮箱数组里
		}
	}
	return USART_RT.rxSize;
}

/*----------------------------------------------------------------------------------------
函数名：void Usart::USART_Transmit(void)
功  能：串口DMA发送函数
----------------------------------------------------------------------------------------*/
void Usart::USART_Transmit(void){
	if(DMA_GetCmdStatus(USART_RT.DMAt_Streamx) == DISABLE){					
		DMA_ClearITPendingBit(USART_RT.DMAt_Streamx, USART_RT.DMA_IT_TCIFx); //开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次
    
		DMA_Cmd(USART_RT.DMAt_Streamx, DISABLE);				             //设置当前计数值前先禁用DMA
		USART_RT.DMAt_Streamx->M0AR = (uint32_t)USART_RT.pTxbuf;  //设置当前待发数据基地址:Memory0 tARget
		USART_RT.DMAt_Streamx->NDTR = (uint32_t)USART_RT.txlen;     //设置当前待发的数据的数量:Number of Data units to be TRansferred
		DMA_Cmd(USART_RT.DMAt_Streamx, ENABLE);
	}
}
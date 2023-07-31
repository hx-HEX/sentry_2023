#pragma once
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "string.h"

#define USART1_RXDMA_LEN 50
#define USART2_RXDMA_LEN 500
#define USART3_RXDMA_LEN 500
#define UART4_RXDMA_LEN 500
#define UART5_RXDMA_LEN 500
#define USART6_RXDMA_LEN 500

extern uint8_t UA1RxDMAbuf[USART1_RXDMA_LEN];
extern uint8_t UA2RxDMAbuf[USART2_RXDMA_LEN];
extern uint8_t UA3RxDMAbuf[USART3_RXDMA_LEN];
extern uint8_t UA4RxDMAbuf[UART4_RXDMA_LEN];
extern uint8_t UA5RxDMAbuf[UART5_RXDMA_LEN];
extern uint8_t UA6RxDMAbuf[USART6_RXDMA_LEN];


#ifdef __cplusplus

class Usart{
public:
    #pragma pack(1)
    //串口DMA收发数据
    struct USART_RTX_TypeDef{
        USART_TypeDef* USARTx;            //串口
        DMA_Stream_TypeDef* DMAr_Streamx; //DMA接收数据流
        uint8_t* pMailbox;                 //邮箱(有效数据)数组
        uint8_t* pDMAbuf;             //DMA数组
        uint16_t MbLen;                   //mailbox长度
        uint16_t DMALen;                  //DMA长度
        uint16_t rxSize;                  //本次接收的长度
        uint16_t rxConter;                //本次DMA长度
	    uint16_t rxBufferPtr;             //上次的长度  长度也代表位置

        DMA_Stream_TypeDef* DMAt_Streamx; //DMA发送数据流
        uint8_t* pTxbuf;                  //发送数组
        uint16_t txlen;                   //发送长度
        uint32_t DMA_IT_TCIFx;            //发送中断标志
        
        uint8_t Usart_event_bit;          //事件标志位
    };
    #pragma pack()

    USART_RTX_TypeDef USART_RT;

    Usart(){
        USART_RT = {0};
    }
    
    void USART_Init(uint8_t* pMailbox,uint16_t MbLen,uint8_t* pTxbuf,uint16_t txlen,uint8_t Usart_event_bit);
    uint16_t USART_Receive(void);
    void USART_Transmit(void);
};

#endif
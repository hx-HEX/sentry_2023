#include "usart.h"
#include "usart_interface.h"

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#endif
/*************************************************************************
函 数 名：USART1_Configuration(void)
函数功能：遥控器接收机DR16底层配置
备    注：PA10(USART1_RX)
*************************************************************************/
void USART1_Configuration(void)
{
    USART_InitTypeDef USART1_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB	| RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7 ,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin      =     GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode     =     GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType    =     GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed    =     GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd     =     GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
      
    USART_DeInit(USART1);
	USART1_InitStructure.USART_BaudRate            =    100000;//SBUS 100K baudrate
	USART1_InitStructure.USART_WordLength          =    USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits            =    USART_StopBits_1;
	USART1_InitStructure.USART_Parity              =    USART_Parity_Even;
	USART1_InitStructure.USART_Mode                =    USART_Mode_Rx;
    USART1_InitStructure.USART_HardwareFlowControl =    USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART1_InitStructure);
    
	USART_Cmd(USART1,ENABLE);//使能串口
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//开启串口DMA接收功能
    
    NVIC_InitStructure.NVIC_IRQChannel                     = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_InitStructure.DMA_Channel                 =    DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr      =    (uint32_t)&(USART1->DR);//设置DMA传输外设基地址
    DMA_InitStructure.DMA_Memory0BaseAddr         =    (uint32_t)UA1RxDMAbuf;//设置DMA传输内存基地址
    DMA_InitStructure.DMA_DIR                     =    DMA_DIR_PeripheralToMemory;//设置数据传输方向
    DMA_InitStructure.DMA_BufferSize              =    USART1_RXDMA_LEN;//设置DMA一次传输数据量的大小,DR16每隔7ms通过DBus发送一帧数据（18字节）
    DMA_InitStructure.DMA_PeripheralInc           =    DMA_PeripheralInc_Disable;//设置外设地址不变
    DMA_InitStructure.DMA_MemoryInc               =    DMA_MemoryInc_Enable;	//设置内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize      =    DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
    DMA_InitStructure.DMA_MemoryDataSize          =    DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
    DMA_InitStructure.DMA_Mode                    =    DMA_Mode_Circular;//设置DMA模式为循环模式
    DMA_InitStructure.DMA_Priority                =    DMA_Priority_VeryHigh;//设置DMA通道的优先级为最高优先级
    DMA_InitStructure.DMA_FIFOMode                =    DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold           =    DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst             =    DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst         =    DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2,&DMA_InitStructure);

    DMA_Cmd(DMA2_Stream2,ENABLE);//使能DMA
}

/*************************************************************************
函 数 名：USART2_Configuration
函数功能：串口2底层配置
备    注：
*************************************************************************/
void USART2_Configuration(void)
{
    USART_InitTypeDef USART2_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);//使能PA端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//IO口速度为100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA,&GPIO_InitStructure);//根据设定参数初始化GPIOA

	/*USART2接收空闲中断*/
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能 
    NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化VIC寄存器

    USART2_InitStructure.USART_BaudRate = 921600;//波特率
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART2_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART2_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//收发模式
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_Init(USART2,&USART2_InitStructure);//初始化串口

	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	
    USART_Cmd(USART2,ENABLE);//使能串口
	//Rx
	DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;//外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)UA2RxDMAbuf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;//DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = USART2_RXDMA_LEN;//设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5,ENABLE);
	
	//Tx
	DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = NULL;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = NULL;            //设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream6,DISABLE);
}

/*************************************************************************
函 数 名：USART3_Configuration
函数功能：串口3底层配置
备    注：
*************************************************************************/
void USART3_Configuration(void)
{
	USART_InitTypeDef USART3_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);//使能PB端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能UART3时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);//根据设定参数初始化GPIOC

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能 
	NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化VIC寄存器

	USART3_InitStructure.USART_BaudRate = 921600;//波特率
	USART3_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART3_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART3_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART3_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//仅接收
	USART3_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART3,&USART3_InitStructure);//初始化串口

	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);

	USART_Cmd(USART3,ENABLE);//使能串口

	//Rx
	DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;//外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);//内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)UA3RxDMAbuf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;//DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = USART3_RXDMA_LEN;//设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream1,ENABLE);
	
	//Tx
	DMA_DeInit(DMA1_Stream3);	
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = NULL;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = NULL;            //设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream3,DISABLE);
}

/*************************************************************************
函 数 名：UART4_Configuration
函数功能：串口4底层配置
备    注：  
*************************************************************************/
void UART4_Configuration(void)
{
	USART_InitTypeDef USART4_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);//使能PC端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);//根据设定参数初始化GPIOC

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能 
	NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化VIC寄存器

	USART4_InitStructure.USART_BaudRate = 460800;//波特率
	USART4_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART4_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART4_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART4_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//仅接收
	USART4_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(UART4,&USART4_InitStructure);//初始化串口

	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);

	USART_Cmd(UART4,ENABLE);//使能串口

	//RX
	DMA_DeInit(DMA1_Stream2);
	DMA_InitStructure.DMA_Channel               = DMA_Channel_4;//通道
	DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(UART4->DR);//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)UA4RxDMAbuf;//将串口4接收到的数据ucRxData_DMA1_Stream2[]里，内存基地址
	DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize            = UART4_RXDMA_LEN;//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;	//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;//DMA_Mode_Normal;////设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority              = DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2, ENABLE);

	//TX
	DMA_DeInit(DMA1_Stream4);
	DMA_InitStructure.DMA_Channel				=	DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)&(UART4->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr		=	NULL;//暂无
	DMA_InitStructure.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	//内存到外设
	DMA_InitStructure.DMA_BufferSize			=	NULL;//暂无
	DMA_InitStructure.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc			    =	DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode				    =	DMA_Mode_Normal;			//正常发送
	DMA_InitStructure.DMA_Priority			    =	DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode			    =	DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold		    =	DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
}

/*************************************************************************
函 数 名：UART6_Configuration
函数功能：串口6底层配置
备    注：备用  
*************************************************************************/
void USART6_Configuration(void)
{
	USART_InitTypeDef USART6_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2,ENABLE);//使能PB端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能UART3时钟

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);//根据设定参数初始化GPIOC

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能 
	NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化VIC寄存器

	USART6_InitStructure.USART_BaudRate = 460800;//波特率
	USART6_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART6_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART6_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART6_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//仅接收
	USART6_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART6,&USART6_InitStructure);//初始化串口

	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);

	USART_Cmd(USART6,ENABLE);//使能串口

	//Rx
	DMA_DeInit(DMA2_Stream1);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;//外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)UA6RxDMAbuf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;//DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = USART6_RXDMA_LEN;//设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream1,ENABLE);
	
	//Tx
	DMA_DeInit(DMA2_Stream6);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5; 
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);     //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = NULL;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //DMA传输为单向
    DMA_InitStructure.DMA_BufferSize         = NULL;            //设置DMA在传输区的长度
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6,&DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream6,DISABLE);
}

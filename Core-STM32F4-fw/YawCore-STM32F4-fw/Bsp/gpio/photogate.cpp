#include "photogate.h"

uint8_t Photogate::Get_Photogate_State(void)
{ 
    return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9) == Bit_RESET ? 1 : 0;
}
#include "photogate.h"

uint8_t Photogate::Get_Photogate_State(void)
{ 
    return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) == Bit_RESET ? 1 : 0;
}
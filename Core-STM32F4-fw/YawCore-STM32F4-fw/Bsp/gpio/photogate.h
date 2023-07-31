#pragma once
#include "stm32f4xx_gpio.h"
#include "stdint.h"

class Photogate
{
public:
    uint8_t calibrate_flag;

    uint8_t Get_Photogate_State(void);
    Photogate(){calibrate_flag = 0;};
};


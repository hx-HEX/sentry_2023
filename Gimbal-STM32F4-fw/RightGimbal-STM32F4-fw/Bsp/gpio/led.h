#pragma once
#include "gpio.h"

#define LED_YELLOW_OFF()    GPIOC->BSRRL = GPIO_Pin_13
#define LED_YELLOW_ON()     GPIOC->BSRRH = GPIO_Pin_13
#define LED_YELLOW_TOGGLE()    GPIOC->ODR ^= GPIO_Pin_13

#define LED_GREEN_OFF()     GPIOC->BSRRL = GPIO_Pin_14
#define LED_GREEN_ON()      GPIOC->BSRRH = GPIO_Pin_14
#define LED_GREEN_TOGGLE()     GPIOC->ODR ^= GPIO_Pin_14

#define LED_BLUE_OFF()      GPIOC->BSRRL = GPIO_Pin_15
#define LED_BLUE_ON()       GPIOC->BSRRH = GPIO_Pin_15
#define LED_BLUE_TOGGLE()      GPIOC->ODR ^= GPIO_Pin_15



class Led {
public:
    void SetGreen(bool state);
    void ToggleGreen(void);
    void SetBlue(bool state);
    void ToggleBlue(void);
    void SetYellow(bool state);
    void ToggleYellow(void);
};
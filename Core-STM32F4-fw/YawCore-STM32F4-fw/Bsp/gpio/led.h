#pragma once
#include "stm32f4xx_gpio.h"

#define  LED_PINK_ON()      	GPIOE->BSRRL = GPIO_Pin_15
#define  LED_PINK_OFF()     	 GPIOE->BSRRH = GPIO_Pin_15
#define  LED_PINK_TOGGLE()  	GPIOE->ODR ^= GPIO_Pin_15

#define  LED_BLUE_ON()      	GPIOE->BSRRL = GPIO_Pin_14
#define  LED_BLUE_OFF()     	GPIOE->BSRRH = GPIO_Pin_14
#define  LED_BLUE_TOGGLE()  	GPIOE->ODR ^= GPIO_Pin_14

#define  LED_GREEN_ON()      	GPIOE->BSRRL = GPIO_Pin_13
#define  LED_GREEN_OFF()     	GPIOE->BSRRH = GPIO_Pin_13
#define  LED_GREEN_TOGGLE()  	GPIOE->ODR ^= GPIO_Pin_13

#define  LED_RED_ON()      		GPIOE->BSRRL = GPIO_Pin_12
#define  LED_RED_OFF()     		GPIOE->BSRRH = GPIO_Pin_12
#define  LED_RED_TOGGLE()  		GPIOE->ODR ^= GPIO_Pin_12

#define  LED_ORANGE_ON()      	GPIOE->BSRRL = GPIO_Pin_11
#define  LED_ORANGE_OFF()     	GPIOE->BSRRH = GPIO_Pin_11
#define  LED_ORANGE_TOGGLE()  	GPIOE->ODR ^= GPIO_Pin_11



class Led {
public:
    void SetPink(bool state);
    void SetBlue(bool state);
    void SetGreen(bool state);
    void SetRed(bool state);
    void SetOrange(bool state);
    void TogglePink(void);
    void ToggleBlue(void);
    void ToggleGreen(void);
    void ToggleRed(void);
    void ToggleOrange(void);
};
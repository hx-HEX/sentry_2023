#include "led.h"



void Led::SetGreen(bool state) {
    if (state) {
        LED_GREEN_ON();
    } else {
        LED_GREEN_OFF();
    }
}


void Led::ToggleGreen(void) {
    LED_GREEN_TOGGLE();
}

void Led::SetBlue(bool state) {
    if (state) {
        LED_BLUE_ON();
    } else {
        LED_BLUE_OFF();
    }
}


void Led::ToggleBlue(void) {
    LED_BLUE_TOGGLE();
}


void Led::SetYellow(bool state) {
    if (state) {
        LED_YELLOW_ON();
    } else {
        LED_YELLOW_OFF();
    }
}


void Led::ToggleYellow(void) {
    LED_YELLOW_TOGGLE();
}
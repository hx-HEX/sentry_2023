#include "led.h"

/**
 * @brief Set pink led state
 *
 */
void Led::SetPink(bool state) {
    if (state) {
        LED_PINK_ON();
    } else {
        LED_PINK_OFF();
    }
}



/**
 * @brief toggle pink led state
 *
 */
void Led::TogglePink(void) {
    LED_PINK_TOGGLE();
}



/**
 * @brief Set blue led state
 *
 */
void Led::SetBlue(bool state) {
    if (state) {
        LED_BLUE_ON();
    } else {
        LED_BLUE_OFF();
    }
}



/**
 * @brief toggle blue led state
 *
 */
void Led::ToggleBlue(void) {
    LED_BLUE_TOGGLE();
}



/**
 * @brief Set green led state
 *
 */
void Led::SetGreen(bool state) {
    if (state) {
        LED_GREEN_ON();
    } else {
        LED_GREEN_OFF();
    }
}



/**
 * @brief toggle green led state
 *
 */
void Led::ToggleGreen(void) {
    LED_GREEN_TOGGLE();
}



/**
 * @brief Set red led state
 *
 */
void Led::SetRed(bool state) {
    if (state) {
        LED_RED_ON();
    } else {
        LED_ORANGE_OFF();
    }
}



/**
 * @brief toggle red led state
 *
 */
void Led::ToggleRed(void) {
    LED_RED_TOGGLE();
}



/**
 * @brief Set orange led state
 *
 */
void Led::SetOrange(bool state) {
    if (state) {
        LED_ORANGE_ON();
    } else {
        LED_ORANGE_OFF();
    }
}



/**
 * @brief toggle orange led state
 *
 */
void Led::ToggleOrange(void) {
    LED_ORANGE_TOGGLE();
}


#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"

#define LED_R GPIO_NUM_25
#define LED_G GPIO_NUM_26
#define LED_B GPIO_NUM_27

#define LED_ON 1
#define LED_OFF 0


void board_led_operation(uint8_t pin, uint8_t onoff);

void board_init(void);

#endif
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#define TAG "BOARD"

#define BLINK_GPIO 14

void board_led_operation(uint8_t pin, uint8_t onoff)
{

    gpio_set_level(BLINK_GPIO, onoff);
}

static void board_led_init(void)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void board_init(void)
{
    board_led_init();
}
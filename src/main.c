/**
 * @file main.c
 * @author Brian Sidebotham <brian.sidebotham@gmail.com>
 * @brief
 * @version 0.1
 * @date 2022-05-06
 *
 * @copyright Copyright (c) Brian Sidebotham 2022
 *
 */

#include "pico/stdlib.h"
#include "spi.h"

int main() {
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}

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
#include "mcp2515.h"
#include "spi.h"
#include <stdio.h>

int main() {
    const uint cs_pin = 17;
    const uint sck_pin = 18;
    const uint mosi_pin = 19;
    const uint miso_pin = 16;
    const uint LED_PIN = 25;

    spi_inst_t *spi = spi0;

    stdio_init_all();

    // Set up the Chip select pin for SPI
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    // Initialise the SPI peripheral
    spi_init(spi, 1000 * 1000);
    spi_set_format( spi, 8, 1, 1, SPI_MSB_FIRST );
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    uint8_t canctrl = MCP2515_read_reg(spi, cs_pin, MCP2515_CANCTRL);

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

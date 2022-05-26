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
    const uint cs_pin = 1;
    const uint sck_pin = 2;
    const uint mosi_pin = 3;
    const uint miso_pin = 4;
    const uint LED_PIN = 25;

    spi_inst_t *spi = spi0;

    stdio_init_all();

    // Set up the Chip select pin for SPI
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialise the SPI peripheral
    spi_init(spi, 1000 * 1000);
    spi_set_format( spi, 8, 1, 1, SPI_MSB_FIRST );
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    /* We must be in configuration mode in order to start configuring the CAN interface*/
    do {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        MCP2515_reset( spi, cs_pin );

        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    } while( MCP2515_get_mode( spi, cs_pin ) != MCP2515_CANCTRL_MODE_CONFIGURATION );

    MCP2515_set_config( spi, cs_pin, 0x00, 0x91, 0x01 );

    gpio_put(LED_PIN, 1);

    while (true) {

    }
}

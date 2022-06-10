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
#include <string.h>

#define CS_PIN              1
#define CAN_MSG_MAX_LENGTH  8
#define LED_PIN             25
#define LED_ON_TICK_COUNT   4

typedef struct {
    uint8_t ready;
    uint8_t id;
    uint32_t eid;
    uint8_t data_length;
    uint8_t data[CAN_MSG_MAX_LENGTH];
} can_msg_t;

static spi_inst_t *spi = spi0;
static uint8_t rx_buffer[13] = {0};
static can_msg_t can_msg = {0};

static int led_on_time = 0;

void mcp2515_interrupt_callback(uint gpio, uint32_t events)
{
    /* Process receiving a message from the MCP2515 CAN Controller */
    uint8_t intreg = MCP2515_read_reg( spi, CS_PIN, MCP2515_CANINTF );
    uint8_t rx_buffer[13];
    uint32_t id = 0;

    if( intreg & MCP2515_CANINTF_RX0IF )
    {
        MCP2515_read_rx_buffer(spi, CS_PIN, 0, &rx_buffer[0]);
        if( rx_buffer[1] & MCP2515_RXBnSIDL_IDE )
        {
            /* Received a frame with an extended ID */
            can_msg.id = 0;
            can_msg.eid = (uint32_t)MCP2515_RXBnSIDL_EID17_16_VALUE(rx_buffer[1]) << 16 |
                          (uint32_t)rx_buffer[2] << 8 |
                          (uint32_t)rx_buffer[3];
            can_msg.data_length = MCP2515_RXBnDLC_DLC3_0_VALUE(rx_buffer[4]);
            memcpy(&can_msg.data[0], &rx_buffer[5], CAN_MSG_MAX_LENGTH);
            can_msg.ready = 1;

            if( can_msg.eid == 0x2000 )
            {
                led_on_time = LED_ON_TICK_COUNT;
            }
        }

        /* Clear the interrupt flags so we can receive another interrupt */
        MCP2515_write_reg(spi, CS_PIN, MCP2515_CANINTF, 0x00);
    }
}

bool tick_callback(struct repeating_timer *t)
{
    if( led_on_time ) {
        gpio_put(LED_PIN, 1);
        led_on_time--;
    } else {
        gpio_put(LED_PIN, 0);
    }
}

int main() {

    const uint sck_pin = 2;
    const uint mosi_pin = 3;
    const uint miso_pin = 4;
    const uint MCP2515_INT_PIN = 16;
    volatile uint8_t intreg = 0;
    struct repeating_timer timer;

    stdio_init_all();

    // Set up the Chip select pin for SPI
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* The GPIO16 pin is connected to the MCP2515 interrupt pin which can then tell us to perform some function to do with the CAN bus */
    gpio_set_irq_enabled_with_callback(MCP2515_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mcp2515_interrupt_callback);

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
        MCP2515_reset( spi, CS_PIN );

        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    } while( MCP2515_get_mode( spi, CS_PIN ) != MCP2515_CANCTRL_MODE_CONFIGURATION );

    /* The configuration for
    /* https://www.kvaser.com/support/calculators/bit-timing-calculator/ */
    MCP2515_set_config( spi, CS_PIN, 0x00, 0x91, 0x01 );

    /* Receive all messages */
    MCP2515_write_reg( spi, CS_PIN, MCP2515_RXB0CTRL, 0x3 << MCP2515_RXB0CTRL_RXM_BIT );
    MCP2515_write_reg( spi, CS_PIN, MCP2515_RXB1CTRL, 0x3 << MCP2515_RXB1CTRL_RXM_BIT );

    /* Enable interrupt pin outputs so I can scope to see if anything is ever received! */
    MCP2515_write_reg( spi, CS_PIN, MCP2515_BFPCTRL, MCP2515_B1BFE | MCP2515_B0BFE | MCP2515_B1BFM | MCP2515_B0BFM );
    MCP2515_write_reg( spi, CS_PIN, MCP2515_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE );

    /* Enter normal operating mode */
    MCP2515_set_mode( spi, CS_PIN, MCP2515_CANCTRL_MODE_NORMAL );

    do {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);

        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        intreg = MCP2515_get_mode( spi, CS_PIN );
    } while( intreg != MCP2515_CANCTRL_MODE_NORMAL );

    /* Add a repeating timer at 10ms no matter how long the timer callback takes to interrupt */
    add_repeating_timer_ms(10, tick_callback, NULL, &timer);

    while (true) {
        tight_loop_contents();
    }
}

/* An example frame received from the DTA S series ECU

 0 :0x00 SIDH
 1: 0x08 SIDL
 2: 0x20 EID8
 3: 0x00 EID0
 4: 0x08 DLC
 5: 0x00 D0
 6: 0x00
 7: 0x19
 8: 0x00
 9: 0x5A
10: 0x00
11: 0x1E
12: 0x00
*/

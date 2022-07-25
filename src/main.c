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

#define CS_PIN                      1
#define CAN_MSG_MAX_LENGTH          8
#define LED_PIN                     25
#define LED_ON_TICK_COUNT           4
#define SCK_PIN                     2
#define MOSI_PIN                    3
#define MISO_PIN                    4
#define MCP2515_INT_PIN             16
#define SWITCH_PIN                  17
#define SWITCH_LED_PIN              15
#define SWITCH_LED_ON_TICK_COUNT    4

typedef struct {
    uint8_t ready;
    uint8_t id;
    uint32_t eid;
    uint8_t data_length;
    uint8_t data[CAN_MSG_MAX_LENGTH];
} can_msg_t;

static spidrv_t* spi = NULL;
static can_msg_t can_msg = {0};

static int led_on_time = 0;

/* The state of the local GPIO pins */
static uint8_t gpio_switches[2] = {0};

/* The state of the switches last received from the CAN bus */
static uint8_t can_switches[2] = {0};

/* Set to non-zero when the MCP2515 interrupt pin has been active */
static volatile int mcp2515_interrupt = 0;

void mcp2515_interrupt_callback(uint gpio, uint32_t events)
{
    /* Only processing interrupts when spi has been configured */
    if( spi == NULL ) {
        return;
    }

    // Flag that the interrupt ocurred and use foreground processing to read the MCP2515 data
    mcp2515_interrupt = 1;
}

void mcp2515_post_interrupt_process() {
    /* Process receiving a message from the MCP2515 CAN Controller */
    uint8_t intreg = MCP2515_read_reg( spi, MCP2515_CANINTF );
    uint8_t rx_buffer[13];
    uint32_t id = 0;
    bool int_cleared = false;

    if( intreg & MCP2515_CANINTF_TX0IF ) {
        MCP2515_bit_modify(
                spi,
                MCP2515_CANINTF,
                MCP2515_CANINTF_TX0IF,
                ~MCP2515_CANINTF_TX0IF );
        int_cleared = true;
    }

    if( intreg & MCP2515_CANINTF_TX1IF ) {
        MCP2515_bit_modify(
                spi,
                MCP2515_CANINTF,
                MCP2515_CANINTF_TX1IF,
                ~MCP2515_CANINTF_TX1IF );
        int_cleared = true;
    }

    if( intreg & MCP2515_CANINTF_TX2IF ) {
        MCP2515_bit_modify(
                spi,
                MCP2515_CANINTF,
                MCP2515_CANINTF_TX2IF,
                ~MCP2515_CANINTF_TX2IF );
        int_cleared = true;
    }

    if( ( intreg & MCP2515_CANINTF_RX0IF ) || ( intreg & MCP2515_CANINTF_RX1IF ) )
    {
        if( intreg & MCP2515_CANINTF_RX0IF ) {
            id = 0;
        } else {
            id = 1;
        }

        MCP2515_read_rx_buffer(spi, id, &rx_buffer[0]);
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
            else if ( ( can_msg.eid > 0x2000 ) && ( can_msg.eid < 0x2006 ) )
            {
                led_on_time = LED_ON_TICK_COUNT;
            }
            else
            {
                /* If we receive switch data on the bus we */
                if( can_msg.eid == 0x4000 )
                {
                    led_on_time = LED_ON_TICK_COUNT;
                    can_switches[0] = can_msg.data[0];
                    can_switches[1] = can_msg.data[0];
                }
            }
        }

        /* Clear the interrupt flag so we can receive another interrupt */
        if( id == 0 ) {
            MCP2515_bit_modify( spi, MCP2515_CANINTF, MCP2515_CANINTF_RX0IF, ~MCP2515_CANINTF_RX0IF );
            int_cleared = true;
        } else {
            MCP2515_bit_modify( spi, MCP2515_CANINTF, MCP2515_CANINTF_RX1IF, ~MCP2515_CANINTF_RX1IF );
            int_cleared = true;
        }
    }
    else
    {
        if( !int_cleared ) {
            /* Some other interrupt source we can't process yet ! */
            MCP2515_write_reg( spi, MCP2515_CANINTF, 0x00 );
        }
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

    return true;
}

int main() {

    volatile uint8_t intreg = 0;
    struct repeating_timer timer;

    stdio_init_all();

    // Set up the Chip select pin for SPI
    gpio_init( CS_PIN );
    gpio_set_dir( CS_PIN, GPIO_OUT );
    gpio_put( CS_PIN, 1 );

    gpio_init( LED_PIN );
    gpio_set_dir( LED_PIN, GPIO_OUT );

    gpio_init( SWITCH_PIN );
    gpio_pull_up( SWITCH_PIN );    gpio_set_dir( SWITCH_PIN, GPIO_IN );

    gpio_init( SWITCH_LED_PIN );
    gpio_set_dir( SWITCH_LED_PIN, GPIO_OUT );

    /* The GPIO16 pin is connected to the MCP2515 interrupt pin which can then tell us to perform some function to do with the CAN bus */
    gpio_init( MCP2515_INT_PIN );
    gpio_pull_up( MCP2515_INT_PIN );
    gpio_set_dir( MCP2515_INT_PIN, GPIO_IN );

    // Initialise the SPI peripheral
    gpio_set_function( SCK_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MOSI_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MISO_PIN, GPIO_FUNC_SPI );
    spi = SPI_init( CS_PIN, spi0, 1000*1000 );
    SPI_set_format( spi, 8, 1, 1, SPI_MSB_FIRST );

    /* We must be in configuration mode in order to start configuring the CAN interface*/
    do {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        MCP2515_reset( spi );

        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    } while( MCP2515_get_mode( spi ) != MCP2515_CANCTRL_MODE_CONFIGURATION );

    /* https://www.kvaser.com/support/calculators/bit-timing-calculator/ */
    MCP2515_set_config( spi, 0x00, 0x91, 0x01 );

    /* Receive all messages, we're not interested in masking the messages, we'll filter
       them in software instead */
    MCP2515_write_reg(
            spi,
            MCP2515_RXB0CTRL,
            0x3 << MCP2515_RXB0CTRL_RXM_BIT );

    MCP2515_write_reg(
            spi,
            MCP2515_RXB1CTRL,
            0x3 << MCP2515_RXB1CTRL_RXM_BIT );

    /* Enable interrupt pin outputs so I can scope to see if anything is ever received! */
    MCP2515_write_reg(
            spi,
            MCP2515_BFPCTRL,
            ( MCP2515_B1BFE |
              MCP2515_B0BFE |
              MCP2515_B1BFM |
              MCP2515_B0BFM ) );

    /* Enable interrupt generation for all transmit and receive buffers */
    MCP2515_write_reg(
            spi,
            MCP2515_CANINTE,
            ( MCP2515_CANINTE_TX0IE |
              MCP2515_CANINTE_TX1IE |
              MCP2515_CANINTE_TX2IE |
              MCP2515_CANINTE_RX0IE |
              MCP2515_CANINTE_RX1IE ) );

    /* Enter normal operating mode */
    MCP2515_set_mode( spi, MCP2515_CANCTRL_MODE_NORMAL );

    do {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);

        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        intreg = MCP2515_get_mode( spi );
    } while( intreg != MCP2515_CANCTRL_MODE_NORMAL );

    /* Start detecting MCP2515 data via interrupt pin, clear the interrupt flags to clear the
       interrupt pin as well so we definitely fire on the next packet received */
    gpio_set_irq_enabled_with_callback( MCP2515_INT_PIN,
                                        GPIO_IRQ_EDGE_FALL,
                                        true,
                                        &mcp2515_interrupt_callback );

    MCP2515_write_reg( spi, MCP2515_CANINTF, 0x00 );

    /* Add a repeating timer at 10ms no matter how long the timer callback takes to interrupt */
    add_repeating_timer_ms(10, tick_callback, NULL, &timer);

    while( true ) {
        tight_loop_contents();

        // Process any MCP2515 activity required
        if( mcp2515_interrupt ) {
            mcp2515_interrupt = 0;
            mcp2515_post_interrupt_process();
        }

        /* Edge detect the change in switch state and transmit a CAN message when it changes
           state to update the remote value of the switches */
        if( !gpio_get( SWITCH_PIN ) ) {
            if( ( gpio_switches[0] & ( 1 << 0) ) == 0 ) {
                gpio_switches[0] |= ( 1 << 0 );
                MCP2515_write_tx_buffer(spi, 0x4000, &gpio_switches[0], 2);
            }
        } else {
            if( gpio_switches[0] & ( 1 << 0) ) {
                gpio_switches[0] &= ~( 1 << 0 );
                MCP2515_write_tx_buffer(spi, 0x4000, &gpio_switches[0], 2);
            }
        }

        /* Tie the LED PIN output to the switch output whether the switch state is received via
           the CAN bus or the GPIO inputs */
        if( can_switches[0] & ( 1 << 0 ) ) {
            gpio_put( SWITCH_LED_PIN, 1 );
        } else {
            gpio_put( SWITCH_LED_PIN, 0 );
        }
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

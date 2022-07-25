
#include "mcp2515.h"
#include "spi.h"
#include <string.h>

void MCP2515_reset( spidrv_t* spi ) {
    uint8_t reset_instruction[] = { MCP2515_INST_RESET };
    SPI_write_bytes( spi, &reset_instruction[0], sizeof( reset_instruction ) );
}

uint8_t MCP2515_read_reg( spidrv_t* spi, uint8_t reg ) {
    uint8_t read_instruction[] = { MCP2515_INST_READ, reg };
    uint8_t result;
    SPI_write_read_bytes( spi, &read_instruction[0], sizeof(read_instruction), &result, sizeof(result) );
    return result;
}

void MCP2515_write_reg(
        spidrv_t* spi,
        const uint8_t reg,
        uint8_t value ) {
    uint8_t write_instruction[] = { MCP2515_INST_WRITE, reg, value };
    SPI_write_bytes( spi, &write_instruction[0], sizeof( write_instruction ) );
}

mcp2515_canctrl_reqop_t MCP2515_get_mode(spidrv_t* spi ) {
    volatile uint8_t canctrl = MCP2515_read_reg( spi, MCP2515_CANCTRL );
    return MCP2515_CANCTRL_REQOP_VALUE( canctrl );
}

void MCP2515_write_tx_buffer(
        spidrv_t* spi,
        uint16_t id,
        const uint8_t* data,
        const uint8_t nbytes
    ) {

    static uint8_t nbuf = 0;
    uint8_t ctrl = 0;
    uint8_t ctrl_registers[] = {
            MCP2515_TXB0CTRL,
            MCP2515_TXB1CTRL,
            MCP2515_TXB2CTRL };

    /* Wait for the TXREQ bit of one of the transmit buffers to be clear
       before we load a message for transmission */
    do {
        ctrl = MCP2515_read_reg(spi, ctrl_registers[nbuf]);
    } while( ctrl & MCP2515_TXBnCTRL_TXREQ );

    /* TODO: Write a non-extended ID where necessary */
    /* TODO: Select the appropriate empty message buffer as necessary */
    uint8_t load_tx_buffer_instruction[] = {
            MCP2515_INST_WRITE,
            /* The SIDH register is the next register after the transmission
               buffer's control register */
            ctrl_registers[nbuf] + 1,
             /* TXBnSIDH */ 0x00,
             /* TXBnSIDL */ MCP2515_TXBSIDL_EXIDE,
             /* TXBnEID8 */ id >> 8,
             /* TXBnEID0 */ id & 0xFF,
             /* TXBnDLC */  nbytes,
             /* DATA PLACEHOLDER */
             0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00
            };

    /* Move the data into the instruction buffer and transfer to the MCP2515 */
    memcpy(&load_tx_buffer_instruction[7], data, nbytes);
    SPI_write_bytes(
            spi,
            &load_tx_buffer_instruction[0],
            7 + nbytes);

    /* Request that the buffer be transmitted. If we have enabled interrupts we'll know when
       this message has been transmitted */
    // MCP2515_write_reg(spi, MCP2515_TXB0CTRL, MCP2515_TXBnCTRL_TXREQ);
    MCP2515_bit_modify(
            spi,
            ctrl_registers[nbuf],
            MCP2515_TXBnCTRL_TXREQ,
            MCP2515_TXBnCTRL_TXREQ);

    do {
        ctrl = MCP2515_read_reg(spi, ctrl_registers[nbuf] );
    } while( ctrl & MCP2515_TXBnCTRL_TXREQ );

    nbuf++;
    if( nbuf > 2 ) {
        nbuf = 0;
    }
}

void MCP2515_read_rx_buffer(
        spidrv_t* spi,
        const uint8_t id,
        uint8_t* buffer ) {

    uint8_t read_rx_buffer_instruction[] = { MCP2515_INST_READ_RX_BUFFER };

    SPI_write_read_bytes(
            spi,
            &read_rx_buffer_instruction[0],
            sizeof(read_rx_buffer_instruction),
            &buffer[0],
            13 );
}

void MCP2515_set_config(
        spidrv_t* spi,
        const uint8_t cnf1,
        const uint8_t cnf2,
        const uint8_t cnf3 ) {
    MCP2515_write_reg( spi, MCP2515_CNF1, cnf1 );
    MCP2515_write_reg( spi, MCP2515_CNF2, cnf2 );
    MCP2515_write_reg( spi, MCP2515_CNF3, cnf3 );
}

void MCP2515_set_mode(
        spidrv_t* spi,
        const mcp2515_canctrl_reqop_t mode ) {
    volatile uint8_t ctrl = MCP2515_read_reg(spi, MCP2515_CANCTRL );
    ctrl = MCP2515_CANCTRL_REQOP_SET( ctrl, mode );
    MCP2515_write_reg( spi, MCP2515_CANCTRL, ctrl );
}

void MCP2515_bit_modify(
        spidrv_t* spi,
        const uint8_t reg,
        const uint8_t mask,
        const uint8_t value )
{
    uint8_t bitmodify_instruction[] = { MCP2515_INST_BIT_MODIFY, reg, mask, value };
    SPI_write_bytes( spi, &bitmodify_instruction[0], sizeof( bitmodify_instruction ) );
}

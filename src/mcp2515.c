
#include "mcp2515.h"
#include "spi.h"

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

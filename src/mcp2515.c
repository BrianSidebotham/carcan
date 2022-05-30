
#include "mcp2515.h"
#include "spi.h"

void MCP2515_reset( spi_inst_t* spi, const uint cs ) {
    uint8_t reset_instruction[1] = { MCP2515_INST_RESET };
    SPI_write_bytes( spi, cs, &reset_instruction[0], sizeof( reset_instruction ) );
}

uint8_t MCP2515_read_reg( spi_inst_t* spi, const uint cs, uint8_t reg ) {
    uint8_t read_instruction[2] = { MCP2515_INST_READ, reg };
    uint8_t result;
    SPI_write_read_bytes( spi, cs, &read_instruction[0], sizeof(read_instruction), &result, sizeof(result) );
    return result;
}

void MCP2515_write_reg(
        spi_inst_t* spi,
        const uint cs,
        const uint8_t reg,
        uint8_t value ) {
    uint8_t write_instruction[3] = { MCP2515_INST_WRITE, reg, value };
    SPI_write_bytes( spi, cs, &write_instruction[0], sizeof( write_instruction ) );
}

mcp2515_canctrl_reqop_t MCP2515_get_mode(spi_inst_t* spi, const uint cs ) {
    volatile uint8_t canctrl = MCP2515_read_reg( spi, cs, MCP2515_CANCTRL );
    return MCP2515_CANCTRL_REQOP_VALUE( canctrl);
}

void MCP2515_read_rx_buffer(
        spi_inst_t* spi,
        const uint cs,
        const uint8_t id,
        uint8_t* buffer ) {
    uint8_t read_rx_buffer_instruction[1] = { MCP2515_INST_READ_RX_BUFFER };

    SPI_write_read_bytes(
            spi,
            cs,
            &read_rx_buffer_instruction[0],
            sizeof(read_rx_buffer_instruction),
            &buffer[0],
            13 );
}

void MCP2515_set_config(
        spi_inst_t* spi,
        const uint cs,
        const uint8_t cnf1,
        const uint8_t cnf2,
        const uint8_t cnf3 ) {
    MCP2515_write_reg( spi, cs, MCP2515_CNF1, cnf1 );
    MCP2515_write_reg( spi, cs, MCP2515_CNF2, cnf2 );
    MCP2515_write_reg( spi, cs, MCP2515_CNF3, cnf3 );
}

void MCP2515_set_mode(
        spi_inst_t* spi,
        const uint cs,
        const mcp2515_canctrl_reqop_t mode ) {
    volatile uint8_t ctrl = MCP2515_read_reg(spi, cs, MCP2515_CANCTRL );
    ctrl = MCP2515_CANCTRL_REQOP_SET( ctrl, mode );
    MCP2515_write_reg( spi, cs, MCP2515_CANCTRL, ctrl );
}

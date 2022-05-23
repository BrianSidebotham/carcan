
#include "mcp2515.h"
#include "spi.h"

uint8_t MCP2515_read_reg( spi_inst_t* spi, const uint cs, uint8_t reg ) {
    uint8_t read_instruction[2] = { MCP2515_INST_READ, reg };
    uint8_t result;
    SPI_write_read_bytes( spi, cs, &read_instruction[0], sizeof(read_instruction), &result, sizeof(result) );
    return result;
}

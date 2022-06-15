#ifndef _BJS_SPI_H
#define _BJS_SPI_H

#include "hardware/spi.h"

#define SPI_MAX_DRIVERS         2
#define SPI_MAX_RX_DATA_SIZE    16
#define SPI_MAX_TX_DATA_SIZE    16

typedef struct {
    uint chip_select;
    spi_inst_t* spihw;
} spidrv_t;

extern spidrv_t* SPI_init(const uint chip_select, spi_inst_t* spi, uint baud_rate);
extern void SPI_write_bytes( spidrv_t* sd, const uint8_t* data, const uint8_t nbytes );
extern void SPI_read_bytes( spidrv_t* sd, uint8_t* data, const uint8_t nbytes );
extern void SPI_write_read_bytes( spidrv_t* sd,
                           const uint8_t* wr_data,
                           const uint8_t wr_nbytes,
                           uint8_t* rd_data,
                           const uint8_t rd_nbytes );
extern void SPI_set_format( spidrv_t* sd, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, spi_order_t order);

#endif

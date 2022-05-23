#ifndef _BJS_SPI_H
#define _BJS_SPI_H

#include "hardware/spi.h"

extern void SPI_write_bytes( spi_inst_t* spi,
                             const uint cs,
                             const uint8_t* data,
                             const uint8_t nbytes );

extern void SPI_read_bytes( spi_inst_t* spi,
                            const uint cs,
                            uint8_t* data,
                            const uint8_t nbytes );

extern void SPI_write_read_bytes( spi_inst_t* spi,
                                  const uint cs,
                                  const uint8_t* wr_data,
                                  const uint8_t wr_nbytes,
                                  uint8_t* rd_data,
                                  const uint8_t rd_nbytes );

#endif

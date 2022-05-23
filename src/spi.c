#include <stdio.h>
#include "spi.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"


/**
 * @brief Write @a nbytes of data to the SPI bus. Chip select is controlled within this function
 *
 * @param spi
 * @param cs
 * @param data
 * @param nbytes
 */
void SPI_write_bytes( spi_inst_t* spi,
                     const uint cs,
                     const uint8_t* data,
                     const uint8_t nbytes )
{
    gpio_put(cs, 0);
    spi_write_blocking(spi, data, nbytes);
    gpio_put(cs, 1);
}


/**
 * @brief Read @a nbytes of data from the SPI bus. Chip select is controlled within this function
 *
 * @param spi
 * @param cs
 * @param data
 * @param nbytes
 */
void SPI_read_bytes( spi_inst_t* spi,
                     const uint cs,
                     uint8_t* data,
                     const uint8_t nbytes )
{
    gpio_put(cs, 0);
    spi_read_blocking(spi, 0x00, data, nbytes);
    gpio_put(cs, 1);
}


/**
 * @brief In a single transaction write @a wr_nbytes and then read @a rd_nbytes into the read buffer
 *
 * @param spi
 * @param cs
 * @param wr_data
 * @param wr_nbytes
 * @param rd_data
 * @param rd_nbytes
 */
void SPI_write_read_bytes( spi_inst_t* spi,
                           const uint cs,
                           const uint8_t* wr_data,
                           const uint8_t wr_nbytes,
                           uint8_t* rd_data,
                           const uint8_t rd_nbytes ) {
    gpio_put(cs, 0);
    spi_write_blocking(spi, wr_data, wr_nbytes);
    spi_read_blocking(spi, 0x00, rd_data, rd_nbytes);
    gpio_put(cs, 1);
}

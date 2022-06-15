#include <stdio.h>
#include "spi.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

static uint spi_index = 0;
static spidrv_t spi_drivers[SPI_MAX_DRIVERS] = {0};

spidrv_t* SPI_init(const uint chip_select, spi_inst_t* spi, uint baud_rate)
{
    if( spi_index > SPI_MAX_DRIVERS ) {
        return NULL;
    }

    spidrv_t* sd = &spi_drivers[spi_index];
    sd->chip_select = chip_select;
    sd->spihw = spi;

    /* Initialise the actual hardware layer */
    spi_init(spi, 1000 * 1000);

    spi_index++;
    return sd;
}


void SPI_write_bytes( spidrv_t* sd, const uint8_t* data, const uint8_t nbytes )
{
    gpio_put(sd->chip_select, 0);
    spi_write_blocking(sd->spihw, data, nbytes);
    gpio_put(sd->chip_select, 1);
}


void SPI_read_bytes( spidrv_t* sd, uint8_t* data, const uint8_t nbytes )
{
    gpio_put(sd->chip_select, 0);
    spi_read_blocking(sd->spihw, 0x00, data, nbytes);
    gpio_put(sd->chip_select, 1);
}


void SPI_write_read_bytes( spidrv_t* sd,
                           const uint8_t* wr_data,
                           const uint8_t wr_nbytes,
                           uint8_t* rd_data,
                           const uint8_t rd_nbytes )
{
    gpio_put(sd->chip_select, 0);
    spi_write_blocking(sd->spihw, wr_data, wr_nbytes);
    spi_read_blocking(sd->spihw, 0x00, rd_data, rd_nbytes);
    gpio_put(sd->chip_select, 1);
}


void SPI_set_format( spidrv_t* sd, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, spi_order_t order)
{
    spi_set_format(sd->spihw, data_bits, cpol, cpha, order);
}

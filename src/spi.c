#include <stdio.h>
#include "spi.h"
#include "hardware/gpio.h"

void spi_write_byte( spi_inst_t *spi, const uint cs, const uint8_t data )
{
    gpio_put(cs, 0);
    spi_write_blocking(spi, &data, 1);
    gpio_put(cs, 1);
}

//Write 1 byte to the specified register
// void reg_write( spi_inst_t *spi, const uint cs,const uint8_t reg, const uint8_t data )
// {
//     uint8_t msg[2];

//     // Construct message (set ~W bit low, MB bit low)
//     msg[0] = 0x00 | reg;
//     msg[1] = data;

//     // Write to register
//     gpio_put(cs, 0);
//     spi_write_blocking(spi, msg, 2);
//     gpio_put(cs, 1);
// }

// SpiDev::SpiDev(int number)
// {

// }

// SpiDev::~SpiDev()
// {

// }

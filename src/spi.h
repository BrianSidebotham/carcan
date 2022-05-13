#ifndef _BJS_SPI_H
#define _BJS_SPI_H

#include "hardware/spi.h"

extern void spi_write_byte( spi_inst_t *spi, const uint cs, const uint8_t data );

// class SpiDev {
// private:
// public:
//     SpiDev();
//     ~SpiDev();
// };

#endif

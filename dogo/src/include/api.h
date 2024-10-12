#ifndef _API_H
#define _API_H

#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include "string.h"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE0          "/dev/spidev0.0"
#define SPI_DEVICE1          "/dev/spidev0.1"

#define SPI_DATA_LEN    132
#define SPI_SPEED       3200000     // 4mhz

typedef struct 
{
    uint8_t tx_buffer[SPI_DATA_LEN];
    uint8_t rx_buffer[SPI_DATA_LEN];

} spi_buffer;

#endif

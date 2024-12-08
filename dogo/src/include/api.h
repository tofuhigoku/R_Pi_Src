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

#include "data_struct.h"

#define SPI_DEVICE0          "/dev/spidev0.0"
#define SPI_DEVICE1          "/dev/spidev0.1"

#define SPI_DATA_LEN    132
#define SPI_SPEED       3200000     // 4mhz

typedef enum
{

    MS_RETURN_OK = 0,
	MS_RETURN_NULLPTR,
	MS_RETURN_INVALID,
	MS_RETURN_FAIL,
	MS_RETURN_NOT_SUPPORTED,
	MS_RETURN_NOT_EQUAL,
}MS_enum_return ;

typedef struct 
{
    uint8_t tx_buffer[SPI_DATA_LEN];
    uint8_t rx_buffer[SPI_DATA_LEN];

} spi_buffer;

MS_enum_return spi_device_init(int* p_fd);
MS_enum_return spi_device_TransmitReceive(int fd, struct spi_ioc_transfer* p_spi_ioc_transfer, int* NumOfsuccessBytes);

uint32_t calculate_xor_checksum(uint32_t* p_data, size_t len);
MS_enum_return Compare_checksum( uint32_t checksum_value1, uint32_t checksum_value2);
MS_enum_return Spi_debug(spi_command_t* p_spi_command, spi_data_t* p_spi_data);
#endif

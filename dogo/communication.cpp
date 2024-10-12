#include <iostream>
#include <string>

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

#include "api.h"

// #define SPI_DATA_LEN    132
// #define SPI_SPEED       3200000     // 4mhz


// uint8_t tx_buffer[SPI_DATA_LEN];
// uint8_t rx_buffer[SPI_DATA_LEN];
spi_buffer spi_buffer_st = {0};
int fd0, fd1;

struct spi_ioc_transfer trx = {
    .tx_buf = (unsigned long)spi_buffer_st.tx_buffer,
    .rx_buf = (unsigned long)spi_buffer_st.rx_buffer,
    .len =SPI_DATA_LEN,
    .speed_hz = SPI_SPEED,
    .delay_usecs = 0,
    .bits_per_word = 0,
    .word_delay_usecs = 0,

};

uint8_t looper;
uint32_t scratch32;

using namespace std;

int main( int argc, char** argv)
{
    int ret =0;
    (void) argc;
    (void) argv;


    for(int i = 0; i < SPI_DATA_LEN; i++) {
        spi_buffer_st.tx_buffer[i] = i;
        // rx_buffer[i] = 0xFF;
    }

    fd0 = open(SPI_DEVICE0, O_RDWR);
    if(fd0 < 0) {
        printf("Could not open the SPI device...\r\n");
        exit(EXIT_FAILURE);
    }

    ret = ioctl(fd0, SPI_IOC_RD_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not read SPI mode...\r\n");
        close(fd0);
        exit(EXIT_FAILURE);
    }

    scratch32 |= SPI_MODE_0;

    ret = ioctl(fd0, SPI_IOC_WR_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not write SPI mode...\r\n");
        close(fd0);
        exit(EXIT_FAILURE);
    }

    ret = ioctl(fd0, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not read the SPI max speed...\r\n");
        close(fd0);
        exit(EXIT_FAILURE);
    }

    scratch32 = SPI_SPEED;

    ret = ioctl(fd0, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not write the SPI max speed...\r\n");
        close(fd0);
        exit(EXIT_FAILURE);
    }
    uint8_t c =0;
    while (1)
    {
        /* code */
        spi_buffer_st.tx_buffer[0] = c++;
        spi_buffer_st.tx_buffer[SPI_DATA_LEN-1] = c++;
        ret = ioctl(fd0, SPI_IOC_MESSAGE(1), &trx);
        if(ret != SPI_DATA_LEN) {
            printf("SPI transfer returned %d...\r\n", ret);
            break;
        }

        printf("Received SPI buffer...\r\n");
        for(int i =0; i < SPI_DATA_LEN ; i++) {
            // printf("0x%02x ",rx_buffer[i]);
            printf("%d ",spi_buffer_st.rx_buffer[i]);
        }
        printf("\r\n");
        usleep(1000);

    }
    

    close(fd0);

    exit(EXIT_SUCCESS);

    return ret;
}
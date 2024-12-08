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

extern "C" {
#include "api.h"
#include "data_struct.h"
}
#include "QuadrupedRobotObject.h"

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
    }

    if(spi_device_init(&fd0) != MS_RETURN_OK)
    {
        exit(EXIT_FAILURE);
    }

    uint8_t c =0;
    while (1)
    {
        /**
         * This section is for spi communication testing
         */
        /* code */
        spi_buffer_st.tx_buffer[0] = c++;
        spi_buffer_st.tx_buffer[SPI_DATA_LEN-1] = c++;
        ret = spi_device_TransmitReceive(fd0, &trx, NULL);
        if(ret != MS_RETURN_OK) {
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

    

    return ret;
}
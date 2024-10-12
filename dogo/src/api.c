#include "api.h"


MS_enum_return spi_device_init(int* p_fd)
{
    MS_enum_return ret = MS_RETURN_OK;
    int fd;
    int local_ret = 0;
    uint32_t scratch32;

    if(p_fd == NULL)
    {
        ret = MS_RETURN_NULLPTR;
    }

    if(ret == MS_RETURN_OK)
    {
        fd = open(SPI_DEVICE0, O_RDWR);
        if(fd < 0) {
            printf("Could not open the SPI device...\r\n");
            ret = MS_RETURN_FAIL;
        }
    }

    if(ret == MS_RETURN_OK)
    {
        local_ret = ioctl(fd, SPI_IOC_RD_MODE32, &scratch32);
        if(local_ret != 0) {
            printf("Could not read SPI mode...\r\n");
            close(fd);
            ret = MS_RETURN_FAIL;
        }
        else
        {
            scratch32 |= SPI_MODE_0;
            local_ret = ioctl(fd, SPI_IOC_WR_MODE32, &scratch32);
            if(local_ret != 0) {
                printf("Could not write SPI mode...\r\n");
                close(fd);
                ret = MS_RETURN_FAIL;
            }
        }
    }

    if(ret == MS_RETURN_OK)
    {
        local_ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32);
        if(local_ret != 0) {
            printf("Could not read the SPI max speed...\r\n");
            close(fd);
            ret = MS_RETURN_FAIL;
        }
        else
        {
            scratch32 = SPI_SPEED;
            local_ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
            if(local_ret != 0) {
                printf("Could not write the SPI max speed...\r\n");
                close(fd);
                ret = MS_RETURN_FAIL;
            }
        }
    }

    if(ret == MS_RETURN_OK)
    {
        *p_fd = fd;
    }


    return ret;
}

MS_enum_return spi_device_TransmitReceive(int fd, struct spi_ioc_transfer* p_spi_ioc_transfer, int* NumOfsuccessBytes)
{
    MS_enum_return ret = MS_RETURN_OK;
    int local_ret = 0;
    if(p_spi_ioc_transfer ==  NULL)
    {
        ret = MS_RETURN_NULLPTR;
    }

    if(ret == MS_RETURN_OK)
    {
        local_ret = ioctl(fd, SPI_IOC_MESSAGE(1), p_spi_ioc_transfer);
        if(local_ret != SPI_DATA_LEN) {
            ret = MS_RETURN_FAIL;
        }
    }
    if(NumOfsuccessBytes != NULL)
    {
        *NumOfsuccessBytes = local_ret;
    }

    return ret;

}



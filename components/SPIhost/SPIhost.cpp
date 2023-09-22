#include <stdio.h>
#include "SPIhost.h"
#include "driver/gpio.h"

esp_err_t initSPIHost (spi_host_device_t host)
{
	esp_err_t ret;
    spi_bus_config_t buscfg={
        .mosi_io_num=PIN_NUM_MOSI,
    	.miso_io_num=PIN_NUM_MISO,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=MAX_TRANSFERSIZE
    };

    ret=spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
	gpio_set_drive_capability((gpio_num_t)PIN_NUM_MOSI,GPIO_DRIVE_CAP_3 );
	gpio_set_drive_capability((gpio_num_t)PIN_NUM_CLK,GPIO_DRIVE_CAP_3 );
    ESP_ERROR_CHECK(ret);

    return ret;
}

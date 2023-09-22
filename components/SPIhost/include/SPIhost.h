#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18

#define MYSPI_HOST HSPI_HOST

#define MAX_TRANSFERSIZE	64
#include "driver/spi_master.h"

esp_err_t initSPIHost (spi_host_device_t host);

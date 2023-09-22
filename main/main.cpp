/*

 . /home/dig/esp/esp-idf/export.sh
 idf.py monitor -p /dev/ttyUSB2

 -s ${openocd_path}/share/openocd/scripts -f interface/ftdi/esp32_devkitj_v1.cfg -f target/esp32.cfg -c "program_esp /mnt/linuxData/projecten/git/thermostaat/SensirionSCD30/build//app.bin 0x10000 verify"

 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mdns.h"

#include "wifiConnect.h"
#include "settings.h"
#include "updateSpiffsTask.h"
#include "main.h"
#include "i2cTask.h"
#include "measureTask.h"


#define ICALFACTOR  	(1.55/262736)
#define VCALFACTOR		(20.1/640801)

#define APPARENTFACTOR  (392.0/13670)
#define ACTIVEFACTOR	(399.0/16200)


static const char *TAG = "main";

esp_err_t init_spiffs(void);

extern bool settingsChanged; // from settingsScreen
uint32_t stackWm[5];
uint32_t upTime;

TaskHandle_t connectTaskh;
TaskHandle_t I2Ctaskh;
TaskHandle_t measureTaskh;


__attribute__((weak)) int getLogScript(char *pBuffer, int count) {
	return 0;
}

__attribute__((weak)) int getRTMeasValuesScript(char *pBuffer, int count) {
	return 0;
}

__attribute__((weak)) int getInfoValuesScript(char *pBuffer, int count) {
	return 0;
}

// ensure after reset back to factory app for OTA
static void setBootPartitionToFactory(void) {
	esp_image_metadata_t metaData;
	esp_err_t err;

	const esp_partition_t *factPart = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, "factory");
	if (factPart != NULL) {
		esp_partition_pos_t factPartPos;
		factPartPos.offset = factPart->address;
		factPartPos.size = factPart->size;

		esp_image_verify(ESP_IMAGE_VERIFY, &factPartPos, &metaData);

		if (metaData.image.magic == ESP_IMAGE_HEADER_MAGIC) {
			ESP_LOGI(TAG, "Setting bootpartition to OTA factory");

			err = esp_ota_set_boot_partition(factPart);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
			}
		}
	}
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
	i2c_port_t i2c_master_port = I2C_MASTER_NUM;

	i2c_config_t conf = { .mode = I2C_MODE_MASTER, .sda_io_num = I2C_MASTER_SDA_IO, .scl_io_num = I2C_MASTER_SCL_IO, .sda_pullup_en = GPIO_PULLUP_DISABLE, // no pullups, externally switched
			.scl_pullup_en = GPIO_PULLUP_DISABLE, .master = I2C_MASTER_FREQ_HZ, .clk_flags = 0 };
	esp_err_t err = i2c_param_config(i2c_master_port, &conf);
	if (err != ESP_OK) {
		return err;
	}
	return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void printTaskInfo() {
	stackWm[0] = uxTaskGetStackHighWaterMark(connectTaskh);
	stackWm[1] = uxTaskGetStackHighWaterMark(measureTaskh);
	stackWm[2] = uxTaskGetStackHighWaterMark(I2Ctaskh);

	ESP_LOGI(TAG, "FreeRTOS stack connect: %4ld measure: %4ld I2C: %4ld", stackWm[0], stackWm[1], stackWm[2]);
	ESP_LOGI(TAG, "freeHeapSize %d\n", xPortGetFreeHeapSize());

	heap_caps_print_heap_info(MALLOC_CAP_DMA);
}

//socket creation failed: Too many open files in system
extern "C" {
void app_main() {
	esp_err_t err;

	bool toggle = false;

	char newStorageVersion[MAX_STORAGEVERSIONSIZE] = { };
	TaskHandle_t otaTaskh;

	esp_rom_gpio_pad_select_gpio(LED_PIN);
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_PIN, 0);

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_LOGI(TAG, "\n **************** start *****************\n");

	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
		ESP_LOGI(TAG, "nvs flash erased");
	}
	ESP_ERROR_CHECK(err);

	ESP_ERROR_CHECK(init_spiffs());
	setBootPartitionToFactory();

	err = loadSettings();

    gpio_install_isr_service(1 << 3);

    ESP_ERROR_CHECK(initSPIHost(MYSPI_HOST));
    initLCD(MYSPI_HOST);
	vTaskDelay(100/portTICK_PERIOD_MS);


	xTaskCreate(&measureTask, "measureTask", 3500, NULL, 2, &measureTaskh);

	wifiConnect(&connectTaskh);

	do {
		toggle = !toggle;
		gpio_set_level(LED_PIN, toggle);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	} while (!connected);

	newStorageVersion[0] = 0;
	xTaskCreate(&updateSpiffsTask, "updateSpiffsTask", 8192, (void*) newStorageVersion, 5, &otaTaskh);

	while (!spiffsUpdateFinised)
		vTaskDelay(1000);

	if (newStorageVersion[0]) {
		strcpy(userSettings.spiffsVersion, newStorageVersion);
		saveSettings();
	}

	while (1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		upTime++;

		if (!connected) {
			toggle = !toggle;
			gpio_set_level(LED_PIN, toggle);
		} else {
			gpio_set_level(LED_PIN, false);

			if (wifiSettings.updated) {
				wifiSettings.updated = false;
				saveSettings();
			}
			if (settingsChanged) {
				settingsChanged = false;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				saveSettings();
			}
		//	stats_display();
		}
	}
}
}


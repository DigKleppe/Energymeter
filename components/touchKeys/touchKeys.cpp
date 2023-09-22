/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

static const char *TAG = "Touch pad";

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (90)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (100)

volatile bool keyIn;
volatile  uint16_t touchValue;

static int threshold;
static void set_thresholds(void)
{
    touch_pad_read_filtered(TOUCH_PAD_NUM2, (uint16_t *)&touchValue);
    threshold = touchValue;
    ESP_LOGI(TAG, "test init: touch pad [%d] val is %d", TOUCH_PAD_NUM2, touchValue);
    //set interrupt threshold.
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_PAD_NUM2, touchValue * 2 / 3));
}


static void read_task(void *pvParameter)
{
	while (1) {
		touch_pad_clear_status();

		uint16_t value = 0;
		touch_pad_read_filtered(TOUCH_PAD_NUM2, &value);
	//	ESP_LOGI(TAG, "value: %d; init val: %d", value, threshold);

		if (value < threshold * TOUCH_THRESH_PERCENT / 100) {
			ESP_LOGI(TAG, "T%d activated!", TOUCH_PAD_NUM2);
			ESP_LOGI(TAG, "value: %d; init val: %d", value, threshold);
			vTaskDelay(200 / portTICK_PERIOD_MS);
			keyIn = true;
		}
		else
			keyIn = false;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


void startTouchKeys(void)
{
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    ESP_ERROR_CHECK(touch_pad_init());
    touch_pad_config(TOUCH_PAD_NUM2, TOUCH_THRESH_NO_USE);  // init only channel2 (GPIO2)
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_set_meas_time( 100,0xFFFF);
    touch_pad_init();
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    set_thresholds();
    xTaskCreate(&read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
}

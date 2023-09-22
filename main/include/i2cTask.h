/*
 * i2cTask.h
 *
 *  Created on: Aug 9, 2021
 *      Author: dig
 *
 *      handles display and TMP117 (reference sensor)
 */

#ifndef MAIN_INCLUDE_I2CTASK_H_
#define MAIN_INCLUDE_I2CTASK_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void I2CTask(void *arg);


typedef struct {
	char * line1;
	char * line2;
	char * line3;
	int timeToDisplay; // in ms
}displayMssg_t;

extern  QueueHandle_t displayMssgbox;

float getTmp117Temperature (void);
float getTmp117AveragedTemperature (void);

#endif /* MAIN_INCLUDE_I2CTASK_H_ */

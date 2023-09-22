/*
 * measureTask.h
 *
 *  Created on: Aug 9, 2021
 *      Author: dig
 */

#ifndef MAIN_INCLUDE_MEASURETASK_H_
#define MAIN_INCLUDE_MEASURETASK_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NR_NTCS			4

#include "cgiScripts.h"
#include "settings.h"

//#define SIMULATE 		1

#define REFAVERAGES		32
#define MAXSTRLEN		16

#define RREF			9975			// in ohms
#define CAP_PIN			GPIO_NUM_16
#define RREF_PIN		GPIO_NUM_21
#define NTC1_PIN		GPIO_NUM_26
#define NTC2_PIN		GPIO_NUM_27
#define NTC3_PIN		GPIO_NUM_17
#define NTC4_PIN		GPIO_NUM_18


#define MEASINTERVAL			 	5  //interval for sensor in seconds
#define LOGINTERVAL					5   //minutes
#define AVGERAGESAMPLES				((LOGINTERVAL * 60)/(MEASINTERVAL))

#define MAXLOGVALUES				((24*60)/LOGINTERVAL)


#define NOCAL 						99999

void measureTask(void *pvParameters);

//extern float temperature[NR_NTCS];
extern  float refTemperature;
extern bool sensorDataIsSend;

extern SemaphoreHandle_t measureSemaphore; // to prevent from small influences on IRQ

typedef struct {
	float temperature[NR_NTCS];
} calValues_t;

typedef struct {
	int32_t timeStamp;
	float temperature[NR_NTCS];
	float refTemperature;
} log_t;

extern log_t tLog[ MAXLOGVALUES];

typedef struct {
	char averegedValue[MAXSTRLEN+1];
	char momentaryValue[MAXSTRLEN+1];

} measValues_t;

extern measValues_t measValues;


int getRTMeasValuesScript(char *pBuffer, int count) ;
int getNewMeasValuesScript(char *pBuffer, int count);
int getLogScript(char *pBuffer, int count);
int getInfoValuesScript (char *pBuffer, int count);
int getCalValuesScript (char *pBuffer, int count);
int saveSettingsScript (char *pBuffer, int count);
int cancelSettingsScript (char *pBuffer, int count);
int calibrateRespScript(char *pBuffer, int count);
int getSensorNameScript (char *pBuffer, int count);
void parseCGIWriteData(char *buf, int received);




#endif /* MAIN_INCLUDE_MEASURETASK_H_ */

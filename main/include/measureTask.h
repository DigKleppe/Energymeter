/*
 * measureTask.h
 *
 *  Created on: Sept 22, 2023
 *      Author: dig
 */

#ifndef MAIN_INCLUDE_MEASURETASK_H_
#define MAIN_INCLUDE_MEASURETASK_H_

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cgiScripts.h"
#include "settings.h"

//#define SIMULATE 		1

#define REFAVERAGES		32
#define MAXSTRLEN		16

#define MEASINTERVAL			 	5  //interval for sensor in seconds
#define LOGINTERVAL					5   //minutes
#define AVGERAGESAMPLES				((LOGINTERVAL * 60)/(MEASINTERVAL))

#define MAXLOGVALUES				((24*60)/LOGINTERVAL)
#define NOCAL 						99999
#define CALCHECKSTR 				"CalTest1"

typedef struct {
	float activePower;
	float apparentPower;
	float totalEnergy;
	float vRMS;
	float iRMS;
	float mainsFrequency;
	float maxWatts;
	float minWatts;
	uint32_t runTime;
} measValues_t;

extern measValues_t measValues;

typedef struct {
	float activePowerCalFact;
	float activePowerOffset;
	float apparentPowerCalFact;
	float apparentPowerOffset;
	float vRMScalFact;
	float vRMSoffset;
	float iRMScalFact;
	float iRMSoffset;
	float calVoltage;
	float calAmps;
	char checkstr[32+1];
}calibrationValues_t;


typedef struct {
	char name[20];
	float * rawValue;
	float * offset;
	float * calValue;
}info_t;

extern calibrationFactors_t calibrationFactors;
extern calibrationFactors_t defaultCalibrationFactors;
extern bool sensorDataIsSend;

typedef struct {
	int32_t timeStamp;
	float momPower;
	float voltage;
	float current;
	float frequency;
} log_t;

extern log_t tLog[ MAXLOGVALUES];

typedef enum calType_t {CALTYPE_OFFSET, CALTYPE_GAIN };

bool parseCalInfo (char *pcParam, const info_t * infoTable , calType_t calType);
void measureTask(void *pvParameters);
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

int resetTotalEnergyScript(char *pBuffer, int count);
int resetRunTimeScript(char *pBuffer, int count);


#endif /* MAIN_INCLUDE_MEASURETASK_H_ */

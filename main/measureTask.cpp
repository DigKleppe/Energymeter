/*
 * measureTask.cpp
 *
 * Created on: Aug 9, 2021
 * Author: dig
 */
#include <string.h>

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "LCDDogm.h"
#include "touchKeys.h"
#include "udpClient.h"
#include "SPIhost.h"

#include "averager.h"
#include "main.h"
#include "measureTask.h"

static const char *TAG = "measureTask";

extern int scriptState;

//static xQueueHandle gpio_evt_queue = NULL;

bool sensorDataIsSend;

measValues_t measValues;
log_t tLog[ MAXLOGVALUES];

int logTxIdx;
int logRxIdx;

Averager refAverager(REFAVERAGES);  // for reference resistor
Averager ntcAverager[NR_NTCS];
Averager refSensorAverager (AVGERAGESAMPLES);

uint8_t err;

uint32_t timeStamp = 1;

void testLog(void) {
	int len;
	char buf[50];
//	logTxIdx = 0;
	for (int p = 0; p < 20; p++) {

		tLog[logTxIdx].timeStamp = timeStamp++;
		for (int n = 0; n < NR_NTCS; n++) {

			tLog[logTxIdx].temperature[n] = p + n;
		}
		tLog[logTxIdx].refTemperature = tmpTemperature; // from I2C TMP117
		logTxIdx++;
		if (logTxIdx >= MAXLOGVALUES )
			logTxIdx = 0;
	}
	scriptState = 0;
	do {
		len = getLogScript(buf, 50);
		buf[len] = 0;
		printf("%s\r",buf);
	} while (len);

	for (int p = 0; p < 5; p++) {

		tLog[logTxIdx].timeStamp = timeStamp++;
		for (int n = 0; n < NR_NTCS; n++) {

			tLog[logTxIdx].temperature[n] = p + n;
		}
		tLog[logTxIdx].refTemperature = tmpTemperature; // from I2C TMP117
		logTxIdx++;
		if (logTxIdx >= MAXLOGVALUES )
			logTxIdx = 0;
	}
	do {
		len = getNewMeasValuesScript(buf, 50);
		buf[len] = 0;
		printf("%s\r",buf);
	} while (len);
	printf("\r\n *************\r\n");
}

int secToTime(int seconds, char *dest) {
	int len = 0;
	int days = seconds / (3600 * 24);
	seconds -= days * (24 * 3600);
	int hours = seconds / 3600;
	seconds -= (hours * 3600);
	int minutes = seconds / 60;
	seconds -= (minutes * 60);
	if (days > 0)
		len = sprintf(dest, "%dd ", days);
	len += sprintf(dest + len, "%02d", hours);
	len += sprintf(dest + len, ":%02d", minutes);
	len += sprintf(dest + len, ":%02d", seconds);
	return len;
}

void measureTask(void *pvParameters) {
	uint32_t tempvRMS = 0;
	uint32_t tempiRMS = 0;
	int32_t tempActivePwr = 0, tempApparentPwr = 0;
	float actWatts;
	char str[20];
	int logDelay = 5; // skip first samples for log , not stable
	int len;

	TickType_t xLastWakeTime;
	int counts = 0;
	int lastminute = -1;
	int logPrescaler = LOGINTERVAL;
	time_t now;
	struct tm timeinfo;

	startTouchKeys();
	initADE (MYSPI_HOST);

	do {
		measValues.runTime++;
		ADE_sampleRMS(&tempvRMS, &tempiRMS);
		measValues.vRMS = tempvRMS * calibrationFactors.vRMScalFact;
		measValues.iRMS = tempiRMS * calibrationFactors.iRMScalFact;
		measValues.mainsFrequency = ADEreadFrequency();

		if (ADE_measureCycles (CLCLESTIMEOUT)) {	// wait until all mainsCycles are sampled
			tempActivePwr = ADE_readActivePower(); // must be the same as VArms at pf = 1
			measValues.activePower = tempActivePwr * calibrationFactors.activePowerCalFactr;
			measValues.totalEnergy += measValues.activePower / (3600.0 * 1000.0); // kwH

			tempApparentPwr = ADE_readApparentPower(); // must be the same as VArms at pf = 1
			measValues.apparentPower = tempApparentPwr * calibrationFactors.apparentPowerCalFact;

			if (logDelay)
				logDelay--;
			else
				addToLog(measValues);

			if (newCalValueReceived) {
				newCalValueReceived = false;
				if (calibrationFactors.calVoltage != 0) {
					calibrationFactors.vRMScalFact = calibrationFactors.calVoltage / tempvRMS;
				}
				if (calibrationFactors.calAmps != 0) {
					calibrationFactors.iRMScalFact = calibrationFactors.calAmps / tempiRMS;
				}
				actWatts = (float) tempvRMS * (float) (float) tempiRMS * calibrationFactors.vRMScalFact * calibrationFactors.iRMScalFact;
				calibrationFactors.activePowerCalFactr = actWatts / (float) tempActivePwr;
				calibrationFactors.apparentPowerCalFact = actWatts / (float) tempApparentPwr;
				saveCalibrationSettings();
			}

			if (measValues.activePower >= 1000)
				len = sprintf(str, "%5dW ", (int) measValues.activePower);
			else
				len = sprintf(str, "%5.1fW ", measValues.activePower);

			if (measValues.totalEnergy >= 100)
				len += sprintf(str + len, "%dkWh", (int) measValues.totalEnergy);
			else
				len += sprintf(str + len, "%1.2fkWh", measValues.totalEnergy);
			vTaskDelay(10);
			writeLine(1, str);
			secToTime(measValues.runTime, str);
			writeLine(2, str);
			ADEspiDummyNeeded = true; // SPI
			vTaskDelay(10);
			if (keyIn)
				resetTotalEnergyScript(NULL, 0);
		} else {
			initADE(MYSPI_HOST); // restart ADE in timeout
			writeLine(1, "Timeout");
			secToTime(measValues.runTime, str);
			writeLine(2, str);
		}

		sprintf(str, "Emeter %d\n\r", touchValue);
		UDPsendMssg(6000, str, strlen(str));

	} while (1);
}



// called from CGI


int getSensorNameScript(char *pBuffer, int count) {
	int len = 0;
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "Actueel,Nieuw\n");
		len += sprintf(pBuffer + len, "%s\n", userSettings.moduleName);
		return len;
		break;
	default:
		break;
	}
	return 0;
}

int getInfoValuesScript(char *pBuffer, int count) {
	int len = 0;
	char str[10];
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "%s\n", "Meting,Actueel,Offset");
		for ( int n =0; n < NR_NTCS;n++) {
			sprintf (str, "Sensor %d", n+1);
			len += sprintf(pBuffer + len, "%s,%3.2f,%3.2f\n", str, lastTemperature[n]-userSettings.temperatureOffset[n], userSettings.temperatureOffset[n]); // send values and offset
		}
		len += sprintf(pBuffer + len, "Referentie,%3.2f,0\n", refSensorAverager.average()/1000.0);
		return len;
		break;
	default:
		break;
	}
	return 0;
}

// only build javascript table

int getCalValuesScript(char *pBuffer, int count) {
	int len = 0;
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "%s\n", "Meting,Referentie,Stel in,Herstel");
		len += sprintf(pBuffer + len, "%s\n", "Sensor 1\n Sensor 2\n Sensor 3\n Sensor 4\n");
		return len;
		break;
	default:
		break;
	}
	return 0;
}

int saveSettingsScript(char *pBuffer, int count) {
	saveSettings();
	return 0;
}

int cancelSettingsScript(char *pBuffer, int count) {
	loadSettings();
	return 0;
}


calValues_t calValues = { NOCAL, NOCAL, NOCAL };
// @formatter:off
char tempName[MAX_STRLEN];

const CGIdesc_t writeVarDescriptors[] = {
		{ "Temperatuur", &calValues.temperature, FLT, NR_NTCS },
		{ "moduleName",tempName, STR, 1 }
};

#define NR_CALDESCRIPTORS (sizeof (writeVarDescriptors)/ sizeof (CGIdesc_t))
// @formatter:on

int getRTMeasValuesScript(char *pBuffer, int count) {
int len = 0;

switch (scriptState) {
case 0:
	scriptState++;

	len = sprintf(pBuffer + len, "%ld,", timeStamp);
	for (int n = 0; n < NR_NTCS; n++) {
		len += sprintf(pBuffer + len, "%3.2f,", lastTemperature[n] -userSettings.temperatureOffset[n]);
	}
#ifdef SIMULATE
	len += sprintf(pBuffer + len, "%3.3f,", lastTemperature[0] + 15);
#else
	len += sprintf(pBuffer + len, "%3.3f,", getTmp117Temperature());
#endif
	return len;
	break;
default:
	break;
}
return 0;
}

// reads averaged values

int getAvgMeasValuesScript(char *pBuffer, int count) {
int len = 0;

switch (scriptState) {
case 0:
	scriptState++;

	len = sprintf(pBuffer + len, "%ld,", timeStamp);
	for (int n = 0; n < NR_NTCS; n++) {
		len += sprintf(pBuffer + len, "%3.2f,", (int) (ntcAverager[n].average() / 1000.0) - userSettings.temperatureOffset[n]);
	}
	len += sprintf(pBuffer + len, "%3.3f\n", getTmp117AveragedTemperature());
	return len;
	break;
default:
	break;
}
return 0;

}
// these functions only work for one user!

int getNewMeasValuesScript(char *pBuffer, int count) {

int left, len = 0;
if (logRxIdx != (logTxIdx)) {  // something to send?
	do {
		len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].timeStamp);
		for (int n = 0; n < NR_NTCS; n++) {
			len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature[n]- userSettings.temperatureOffset[n]);
		}
		len += sprintf(pBuffer + len, "%3.3f\n", tLog[logRxIdx].refTemperature);
		logRxIdx++;
		if (logRxIdx > MAXLOGVALUES)
			logRxIdx = 0;
		left = count - len;

		} while ((logRxIdx != logTxIdx) && (left > 40));

	}
	return len;
}
// reads all avaiable data from log
// issued as first request.

int getLogScript(char *pBuffer, int count) {
	static int oldTimeStamp = 0;
	static int logsToSend = 0;
	int left, len = 0;
	int n;
	if (scriptState == 0) { // find oldest value in cyclic logbuffer
		logRxIdx = 0;
		oldTimeStamp = 0;
		for (n = 0; n < MAXLOGVALUES; n++) {
			if (tLog[logRxIdx].timeStamp < oldTimeStamp)
				break;
			else {
				oldTimeStamp = tLog[logRxIdx++].timeStamp;
			}
		}
		if (tLog[logRxIdx].timeStamp == 0) { // then log not full
			// not written yet?
			logRxIdx = 0;
			logsToSend = n;
		} else
			logsToSend = MAXLOGVALUES;
		scriptState++;
	}
	if (scriptState == 1) { // send complete buffer
		if (logsToSend) {
			do {
				len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].timeStamp);
				for (n = 0; n < NR_NTCS; n++) {
					len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature[n] - userSettings.temperatureOffset[n]);
				}
				len += sprintf(pBuffer + len, "%3.3f\n", tLog[logRxIdx].refTemperature);
				logRxIdx++;
				if (logRxIdx >= MAXLOGVALUES)
					logRxIdx = 0;
				left = count - len;
				logsToSend--;

			} while (logsToSend && (left > 40));
		}
	}
	return len;
}


// values of setcal not used, calibrate ( offset only against reference TMP117
void parseCGIWriteData(char *buf, int received) {
	if (strncmp(buf, "setCal:", 7) == 0) {  //
		float ref = (refSensorAverager.average() / 1000.0);
		for ( int n = 0; n < NR_NTCS; n++){
			if (lastTemperature[n] != ERRORTEMP ){
				float t =  ntcAverager[n].average() / 1000.0;
				userSettings.temperatureOffset[n] = t - ref;
			}
		}
	} else {
		if (strncmp(buf, "setName:", 8) == 0) {
			if (readActionScript(&buf[8], writeVarDescriptors, NR_CALDESCRIPTORS)) {
				if (strcmp(tempName, userSettings.moduleName) != 0) {
					strcpy(userSettings.moduleName, tempName);
					ESP_ERROR_CHECK(mdns_hostname_set(userSettings.moduleName));
					ESP_LOGI(TAG, "Hostname set to %s", userSettings.moduleName);
					saveSettings();
				}
			}
		}
	}
}



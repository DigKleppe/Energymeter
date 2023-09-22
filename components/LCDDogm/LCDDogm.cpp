#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "LCDDogm.h"

#define PIN_NUM_CS   0
#define PIN_NUM_DC   GPIO_NUM_17  // RS

#define SET_DD_RAM_ADDR	0x80
#define SET_CONTRAST 	0x70

#define DISPLAYCHAR 		16

char cDispLine1[DISPLAYCHAR + 1];
char cDispLine2[DISPLAYCHAR + 1];

static spi_device_handle_t LCDspi;

//uint8_t const bLCDinitStr[] = { 0x39, 0x1C, 0x52, 0x69, 0x74, 0x0C, 0x06, 0x01,	00 }; // 5V  zie datasheet EA-DOGM162 ,00 is einde tabel
uint8_t const bLCDinitStr[] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38, 0x0C, 0x01, 0x06, 00 }; //  3V3 zie datasheet EA-DOGM162 ,00 is einde tabel

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(const uint8_t cmd) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                     //Command is 8 bits
	t.tx_buffer = &cmd;               //The data is the cmd itself
	t.user = (void*) 0;                //D/C needs to be set to 0
	ret = spi_device_polling_transmit(LCDspi, &t);  //Transmit!
	assert(ret == ESP_OK);            //Should have had no issues.
}

void lcdCmd(const uint8_t *cmds, int len) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = len * 8;                     //Command is 8 bits
	t.tx_buffer = cmds;               //The data is the cmd itself
	t.user = (void*) 0;                //D/C needs to be set to 0

	ret = spi_device_polling_transmit(LCDspi, &t);  //Transmit!
	assert(ret == ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data( const uint8_t *data, int len) {
	esp_err_t ret;
	spi_transaction_t t;
	if (len == 0)
		return;             //no need to send anything
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = len * 8;        //Len is in bytes, transaction length is in bits.
	t.tx_buffer = data;               //Data
	t.user = (void*) 1;                //D/C needs to be set to 1
	ret = spi_device_polling_transmit(LCDspi, &t);  //Transmit!
	assert(ret == ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
	int dc = (int) t->user;
	gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
void initLCD( spi_host_device_t host) {
	uint8_t *bp;
	esp_err_t ret;

    spi_device_interface_config_t devcfg={
    	.mode=3,                              //SPI mode 3 clk H idle rising edge
     //  .clock_speed_hz=5*1000*1000,           //Clock out at 5 MHz
		 .clock_speed_hz=100*1000,        //Clock out at 100 kHz due to timing controller
        .spics_io_num=PIN_NUM_CS,              //CS pin
        .queue_size=7,                         //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    ret=spi_bus_add_device(host, &devcfg, &LCDspi);
    ESP_ERROR_CHECK(ret);

	//Initialize non-SPI GPIOs
	gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);

	vTaskDelay(40/portTICK_PERIOD_MS); // 40 ms delay from powerup
	lcdCmd( (uint8_t*) bLCDinitStr, (sizeof bLCDinitStr)-1);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

void clrDisplay(void) {
	lcd_cmd(1);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

void setContrast(uint8_t contrast) {
	lcd_cmd(SET_CONTRAST + contrast);
}


void gotoXY(uint8_t xpos, uint8_t ypos) {
	switch (ypos) {
	case 0:
		lcd_cmd( SET_DD_RAM_ADDR + xpos);
		break;
	case 1:
		lcd_cmd( SET_DD_RAM_ADDR + 0x40 + xpos);
		break;
	case 2:
		lcd_cmd( SET_DD_RAM_ADDR + DISPLAYCHAR + xpos);
		break;
	case 3:
		lcd_cmd( SET_DD_RAM_ADDR + 0x40 + DISPLAYCHAR + xpos);
		break;
	}
}

int writeDisplay(uint8_t chr) {
	switch (chr) {
	case '\1':
		lcd_cmd(SET_DD_RAM_ADDR); /* home commando */
		break;
	case '\2':
	case '\n':
		lcd_cmd(0xC0); /* new line */
		break;
	case '\r':
		lcd_cmd(0x01); /* clr display */
		vTaskDelay(2/portTICK_PERIOD_MS);
		break;
//	case '©':
//		lcd_cmd(223); // graden
//		break;
	default:
		lcd_data (&chr,1);
	}
	return (chr);
}

void displayString(char *str) {
	while (*str)
		writeDisplay(*str++);
}

void writeLine ( int line , char * str) {
	char lin[DISPLAYCHAR];
	int len = strlen (str);
	if (len < DISPLAYCHAR) {
		memcpy( lin,str, strlen(str));
		memset( &lin[len], ' ', DISPLAYCHAR-len); // fill with spaces
	}
	else
		memcpy( lin,str, DISPLAYCHAR);

	switch (line){
	case 1:
		lcd_cmd(SET_DD_RAM_ADDR); /* home commando */
		break;
	case 2:
		lcd_cmd(0xC0); /* new line */
		break;
	}
	lcd_data((uint8_t*) lin , DISPLAYCHAR);
}



#include "driver/spi_master.h"

#define nCSB	0	 // not chip select LCD
#define RS		1

#define DISPLAYCHAR 		16

extern char cDispLine1[DISPLAYCHAR+1];
extern char cDispLine2[DISPLAYCHAR+1];

void initLCD( spi_host_device_t host);
void writeLine ( int line , char * str);

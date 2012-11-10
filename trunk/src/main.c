/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO/STM32
**                STMicroelectronics STM32Lxx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/
/* Includes */
#include <stddef.h>
#include <stdio.h>
#include "mcu.h"
#include "usart.h"
#include "temp_calibrate.h"
#include "i2c.h"
#include "eeprom.h"
#include "mcp9808.h"

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int tick = 0;

//this is a function handling received data
//it is not called automaticaly
void handleReceivedChar(unsigned char data)
{
}


int main(void)
{
	char s[1024];
	uint16_t raw_temperature;
	uint16_t tmp1, tmp2;

	/**
	 * Connection
	 * UART2
	 * pa3		rx
	 * pa2		tx
	 */

	//init peripheries
	initUSART2();
	PutsUART2("Initializing...\n");
	temperature_init();
	eeprom_init();
	initI2C2();

	//init with calibrated data
	tmp1 = read_eeprom_16(EEPROM_START_ADDRESS);
	tmp2 = read_eeprom_16(EEPROM_START_ADDRESS+2);
	if(tmp1 != 0xffffffff && tmp2 != 0xffffffff) {
		temperature_calibrate(tmp1, tmp2);
	}

    while(1)
    {
    	//standard beh
    	raw_temperature = temperature_raw();
    	eeprom_log_next(0, raw_temperature);

    	sprintf(s,"T: %d - %f - %d\n",raw_temperature, temperature(raw_temperature),mcp9808_gettemp(0x18));
    	PutsUART2(s);

    	// replace with sleep
    	delay_us(100000);
    }

	return 0;
}


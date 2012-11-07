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
	/**
	 * Zapojenie
	 * UART1
	 * fialova	pa10	rx
	 * hneda	pa9		tx
	 *
	 * UART2
	 * fialova	pa3		rx
	 * hneda	pa2		tx
	 *
	 * UART3
	 * fialova	pc11	rx
	 * hneda	pc10	tx
	 */
	initUSART2();
	PutsUART2("Initializing...\n");
	temperature_init();

    while(1)
    {
    		sprintf(s,"T: %d - %f\n",temperature_raw(), temperature());
    		PutsUART2(s);
    	    delay_us(100000);
    }

	return 0;
}


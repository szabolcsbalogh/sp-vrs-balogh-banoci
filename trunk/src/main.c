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
#include <stdint.h>
#include "mcu.h"
#include "usart.h"
#include "i2c.h"
#include "eeprom.h"
#include "mcp9808.h"
#include "temperature.h"

#define MCP_ADDR 0x30
#define CAL_NUM 30

int follow_stream = 0;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int tick = 0;
void save_cal_data();

double temp_o, raw_o;
int cal_o = 0;

void calibrate()
{
	char s[512];
	double temp, raw;
	double pp = 0.0, qq = 0.0;

	sprintf(s,"Stage %d...\n",(cal_o + 1));
	PutsUART1(s);
	raw = temperature_raw();
	temp = mcp9808_gettemp(MCP_ADDR);
	sprintf(s,"Meas: %f %f\n",raw, temp);
	PutsUART1(s);
	if(double2data(temp) == 0xffffffffffffffff) {
		sprintf(s,"Error - no data...\n");
		PutsUART1(s);
		return;
	}

	if(cal_o == 1) {
		if(raw_o == raw) {
			sprintf(s,"Second measurement should be with another temperature.\n");
			PutsUART1(s);
		}else{
			pp += (temp - temp_o)/(raw - raw_o);
			qq += temp_o - raw_o * pp;
			sprintf(s,"Saving data p=%f, q=%f ...\n",pp,qq);
			PutsUART1(s);
			temperature_calibrate(pp,qq);
			save_cal_data();
			raw_o = raw;
			temp_o = temp;
		}
	}else{
		cal_o = 1;
		raw_o = raw;
		temp_o = temp;
	}

}

void save_cal_data()
{
	double tmp;
	tmp = temperature_getp();
	write_eeprom_32(EEPROM_START_ADDRESS, double2data(tmp) >> 32 & 0xffffffff);
	write_eeprom_32(EEPROM_START_ADDRESS + 4, double2data(tmp) & 0xffffffff);
	tmp = temperature_getq();
	write_eeprom_32(EEPROM_START_ADDRESS + 8, double2data(tmp) >> 32 & 0xffffffff);
	write_eeprom_32(EEPROM_START_ADDRESS + 12, double2data(tmp) & 0xffffffff);
}

void get_csv() {
	uint32_t addr;
	char s[512];
	sprintf(s,"===CSV START===\n");
	PutsUART1(s);
	//find the start
	for(addr = EEPROM_LOG_START_ADDRESS; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4);
	//put data
	for(; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4) {
		sprintf(s,"%d,%d,%f\n", (uint16_t)read_eeprom_16(addr), (uint16_t)read_eeprom_16(addr+2), temperature((uint16_t)read_eeprom_16(addr+2)));
		PutsUART1(s);
	}
	for(addr = EEPROM_LOG_START_ADDRESS; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4) {
		sprintf(s,"%d,%d,%f\n", (uint16_t)read_eeprom_16(addr), (uint16_t)read_eeprom_16(addr+2),temperature((uint16_t)read_eeprom_16(addr+2)));
		PutsUART1(s);
	}
	sprintf(s,"===CSV END===\n");
	PutsUART1(s);
}

void handleReceivedChar(unsigned char data)
{
	char s[128];
	double temp;
	switch(data) {
	case 'h':
		sprintf(s,"F - flow ON\nf - flow off\ng - get CSV\ne - erase data from EEPROM\n");
		PutsUART1(s);
		sprintf(s,"c - print calibrate data\nr - reset callibration data to 1,0\nC - callibrate\n");
		PutsUART1(s);
		sprintf(s,"i - print I2C temperature\n");
		PutsUART1(s);
		break;
	case 'i':
		temp = mcp9808_gettemp(MCP_ADDR);
		sprintf(s,"Meas: %f %s\n",temp,(double2data(temp) == 0xffffffffffffffff ? "Err.":"Ok."));
		PutsUART1(s);
		break;
	case 'F':
		follow_stream = 1;
		break;
	case 'f':
		follow_stream = 0;
		break;
	case 'g':
		get_csv();
		break;
	case 'c':
		sprintf(s,"p=%f, q=%f\n",temperature_getp(),temperature_getq());
		PutsUART1(s);
		break;
	case 'e':
		sprintf(s,"===EEPROM Clear start====\n");
		PutsUART1(s);
		eeprom_clear();
		sprintf(s,"===EEPROM Clear finished====\n");
		PutsUART1(s);
		break;
	case 'r':
		temperature_calibrate(1.0, 0.0);
		save_cal_data();
		break;
	case 'C':
		sprintf(s,"===Calibrate start====\n");
		PutsUART1(s);
		calibrate();
		sprintf(s,"===Calibrate end====\n");
		PutsUART1(s);
		break;
	}
}

int main(void)
{
	char s[1024];
	uint16_t raw_temperature;
	uint64_t tmp1, tmp2;

	/**
	 * Connection
	 * UART2
	 * pa9
	 * pa10
	 *
	 * yellow scl
	 * blue sdl
	 */

	//init peripheries
	initUSART1();
	PutsUART1("Initializing...\n");
	temperature_init();
	eeprom_init();
	initI2C1();
	RegisterCallbackUART1(&handleReceivedChar);

	//init with calibrated data
	tmp1 = ((uint64_t)read_eeprom_32(EEPROM_START_ADDRESS) << 32) + (uint64_t)read_eeprom_32(EEPROM_START_ADDRESS+4);
	tmp2 = ((uint64_t)read_eeprom_32(EEPROM_START_ADDRESS+8) << 32) + (uint64_t)read_eeprom_32(EEPROM_START_ADDRESS+12);
	temperature_calibrate(data2double(tmp1), data2double(tmp2));

    while(1)
    {
    	//temperature logging
    	raw_temperature = temperature_raw();
    	eeprom_log_next(0, raw_temperature);
    	if(follow_stream) {
    		sprintf(s,"T: %f %d\n",temperature(raw_temperature),raw_temperature);
    		PutsUART1(s);
    	}

    	// replace with sleep
    	delay_us(100000);
    }

	return 0;
}


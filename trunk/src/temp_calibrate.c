#include "stm32l1xx_i2c.h"
#include "temp_calibrate.h"
#include "temperature.h"

uint8_t temp_c_do(uint8_t no, double *p, double *q)
{
	double temp, raw;
	double temp_o, raw_o;
	double pp = 0.0, qq = 0.0;
	int i;
	//init i2c

	raw = temperature_raw();
	//temp = read_i2c_temp;
	for(i=0; i<no; i++) {
		raw_o = raw;
		temp_o = temp;
		raw = temperature_raw();
		//temp = read_i2c_temp;
		pp += (temp - temp_o)/(raw - raw_o);
		qq += temp_o - raw_o * pp;
	}
	pp /= no;
	qq /= no;

	*p = pp;
	*q = qq;

	//deinit i2c
	return TEMP_C_OK;
}

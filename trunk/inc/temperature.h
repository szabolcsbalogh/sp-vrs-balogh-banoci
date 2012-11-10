#include "mcu.h"
#include "stm32l1xx_adc.h"

void temperature_init();
void temperature_calibrate(double p, double q);
uint16_t temperature_raw();
double temperature(uint16_t raw);

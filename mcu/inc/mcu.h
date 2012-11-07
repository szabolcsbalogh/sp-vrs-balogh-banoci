/**
  ******************************************************************************
  * @file    firmware/src/mcu/mcu.h
  * @author  Jozef Rodina
  * @version V1.0.0
  * @date    20-January-2012
  * @brief   MCU file - Some kind of HAL.
  ******************************************************************************
  *
  * <h2><center>&copy; COPYRIGHT 2012 Jozef Rodina</center></h2>
  ******************************************************************************
  */
#ifndef __MCU_H
#define __MCU_H

#include "stm32l1xx.h"
#include "stm32l1xx_conf.h"
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_spi.h"

void delay_us(unsigned long us);

void SwBreak(void);

#endif

/*
 * usart.h
 *
 *  Created on: Sep 26, 2012
 *      Author: lupal
 */

#ifndef USART_H_
#define USART_H_

#include "mcu.h"

void initUSART2(void);

void PutsUART2(char *str);
void PutcUART2(char c);
void PutbUART2(char *str, unsigned short length);

void RegisterCallbackUART2(void *callback);

#endif /* USART_H_ */

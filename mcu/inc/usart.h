/*
 * usart.h
 *
 *  Created on: Sep 26, 2012
 *      Author: lupal
 */

#ifndef USART_H_
#define USART_H_

#include "mcu.h"

void initUSART1(void);
void initUSART2(void);
void initUSART3(void);

void PutsUART1(char *str);
void PutsUART2(char *str);
void PutsUART3(char *str);
void PutcUART1(char c);
void PutcUART2(char c);
void PutcUART3(char c);
void PutbUART1(char *str, unsigned short length);
void PutbUART2(char *str, unsigned short length);
void PutbUART3(char *str, unsigned short length);

void RegisterCallbackUART1(void *callback);
void RegisterCallbackUART2(void *callback);
void RegisterCallbackUART3(void *callback);

#endif /* USART_H_ */

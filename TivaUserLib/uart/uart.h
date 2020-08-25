/*
 * uart.h
 *
 *  Created on: Mar 23, 2018
 *      Author: Tra Duc Toan
 */

#ifndef TIVAUSERLIB_UART_UART_H_
#define TIVAUSERLIB_UART_UART_H_

#include "../userLib.h"

#define size 20
extern void reverse(char *str, int len);
extern int intToStr(int x, char str[], int d);
extern void ftoa(double n, char *res, int afterpoint);
extern void print_num(uint32_t ui32Base ,double numb );

extern void reset_buffer(unsigned char *pBuff);
extern void UARTGetBuffer(unsigned char *pBuff);
extern void Read_UART();


#endif /* TIVAUSERLIB_UART_UART_H_ */

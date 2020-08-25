/*
 * uart.c
 *
 *  Created on: Mar 23, 2018
 *      Author: Tra Duc Toan
 */
#include "../userLib.h"
#include "uart.h"

/*********************write uart*********************/


void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}


void ftoa(double n, char *res, int afterpoint)
{
    // Extract integer part
    long ipart = (long)n;

    // Extract floating part
    double fpart = n - (double)ipart;

    // convert integer part to string
    long i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((long)fpart, res + i + 1, afterpoint);
    }
}

void print_num(uint32_t ui32Base ,double numb )
{
    char  output[size];
    int i=0;
    if(numb<0)
        {
        numb=-numb;
        UARTCharPut(ui32Base, '-');
        }
    if(numb<1 && numb>-1)
        UARTCharPut(ui32Base, '0');

    ftoa(numb,output,3);

    for(i=0;i<size;i++)
    {
        if((output[i]>=48&&output[i]<=57)||output[i]==46)
        UARTCharPut(ui32Base, output[i]);
    }

}


/*********************Read uart*********************/

void reset_buffer(unsigned char *pBuff)
{
    while(*pBuff!=0x0A)//if ending character ==0x0A, put 0 in all buffers.
    {
        *pBuff=0;
        pBuff++;
    }
}

void UARTGetBuffer(unsigned char *pBuff)
{
    static uint16_t i=0;
    if(i==0)
        reset_buffer(pBuff);

    char c;
    while(UARTCharsAvail(UART0_BASE))
    {
        c=UARTCharGet(UART0_BASE);
        *(pBuff+i)=c;
        i++;
    }

    if(c==0x0A)
        i=0;
}

static uint8_t Buff[30];
void Read_UART()
{
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, true));//clear flag
    UARTGetBuffer(&Buff[0]);
}

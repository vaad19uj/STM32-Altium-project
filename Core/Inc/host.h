//----------------------------------
//Header file with functions for
//host communication.
//It uses the hardware USART
//in the MSP430.
//----------------------------------
#include <stdio.h>

void kputchar(char TXchar);
unsigned char Nibble2Ascii (unsigned char anibble);
void Put_byte(unsigned char abyte);

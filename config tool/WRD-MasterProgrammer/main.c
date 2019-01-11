/*
 * WRD-MasterProgrammer.c
 *
 * Created: 1/11/2019 12:12:51 PM
 * Author : Cobus Hoffmann
 */ 
#define F_CPU 8000000ul
#include <avr/io.h>
#include <util/delay.h>
#include "Drivers2/serial.h"


int main(void)
{
    /* Replace with your application code */
	serialInit(9600);
	unsigned char myBuffer[20];
	volatile unsigned char myFlag = 0;
	
	//Set buffer and flag for the serial stuff
	serialSetRxBuffer(myBuffer);
	serialSetRxCompleteFlag(&myFlag);
	//Enable the reception via interrupt
	serialEnableInterrupt();
    while (1) 
    {
		if (myFlag)
		{
			//Serial data has been received
			serialPrintLn("Received:");
			serialPrintLn(myBuffer);
		}
		_delay_ms(1000);
		serialTransmitByte('H');
		serialTransmitByte('\n');
    }
}


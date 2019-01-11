/*
 * WRD-MasterProgrammer.c
 *
 * Created: 1/11/2019 12:12:51 PM
 * Author : Cobus Hoffmann
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "../../library/nrf24l01.h"
#include "../../library/serial.h"


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
    }
}


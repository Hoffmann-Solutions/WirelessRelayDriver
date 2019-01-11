/*
 * WRD-MasterProgrammer.c
 *
 * Created: 1/11/2019 12:12:51 PM
 * Author : Cobus Hoffmann
 */ 
#define F_CPU		16000000ul
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "serial.h"

const char CHECK_OK_CMD[] = "AT";
const char SET_ADDR_CMD[] = "AT+SA:";
const char SET_MODE_CMD[] = "ST+SM:";
int myAddr = 0;
int myMode = 0;

int main(void)
{
    /* Replace with your application code */
	serialInit(MYUBRR);
	char myBuffer[20];
	volatile char myFlag = 0;
	
	memset(myBuffer, '\0', sizeof(myBuffer));
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
			serialPrint("> ");
			serialPrintLn(myBuffer);
			myFlag = 0;
			//Check if the command received matches SET_ADDR_CMD
			if(!strncmp(CHECK_OK_CMD, myBuffer, strlen(CHECK_OK_CMD))){
				//Check command
				serialPrintLn("OK");
			}else if(!strncmp(SET_ADDR_CMD, myBuffer, strlen(SET_ADDR_CMD))){
				//Address command received 
				int addr = atoi(myBuffer+strlen(SET_ADDR_CMD));
				myAddr = addr;
			}else if(!strncmp(SET_MODE_CMD, myBuffer, strlen(SET_MODE_CMD))){
				//Mode command received
				int mode = atoi(myBuffer+strlen(SET_MODE_CMD));
				myMode = mode;
				
			}
			
		}
    }
}


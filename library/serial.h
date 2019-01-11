/*
 * serial.h
 *
 * Created: 1/11/2019 12:16:37 PM
 *  Author: Cobus Hoffmann
 *
 *	Testing the link
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR (unsigned int) (FOSC/16/BAUD-1)

/**
 * \brief This method is used to initialise the serial interface
 * 8data bits and 2 stop bits are used in the message frame
 * 
 * \param baud
 * 
 * \return void
 */
void serialInit(unsigned int ubrr);


/**
 * \brief This function is used to transmit a single byte 
 * over the serial interface
 * 
 * \param data
 * 
 * \return void
 */
void serialTransmitByte(unsigned char data);

/**
 * \brief This function is used to print a '\0' terminated string 
 * via the serial interface
 * 
 * 
 * \return void
 */
void serialPrint(char* string);

/**
 * \brief This function is used to transmit a '\0' terminated
 * string or charachter array over the serial interface.
 * 
 * \param string
 * 
 * \return void
 */
void serialPrintLn(char* string);




/**
 * \brief This method is used to read a byte from the serial 
 * interface
 * 
 * 
 * \return unsigned char
 */
char serialReceiveByte();


/**
 * \brief This function is used to enable the data received interrupt
 * on the serial interface
 * 
 * 
 * \return void
 */
void serialEnableInterrupt();


void serialSetRxBuffer(unsigned char* buff);

void serialSetRxBufferSize(unsigned int numBytes);

void serialSetRxCompleteFlag(unsigned char* flag);






#endif /* SERIAL_H_ */
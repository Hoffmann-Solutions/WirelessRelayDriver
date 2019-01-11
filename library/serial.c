/*
 * serial.c
 *
 * Created: 1/11/2019 12:16:48 PM
 *  Author: Cobus Hoffmann
 */ 

#include "serial.h"

volatile unsigned char* serialRxBuffer;
unsigned int serialRxBufferSize=0;
volatile unsigned char* serialRxCompleteFlag;
unsigned int serialNumBytesReceived = 0;

void serialInit(unsigned int ubrr){
	//Set the baud rate
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	//Set the frame format: 8data, 2stop bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

}


void serialTransmitByte(unsigned char data){
	//Wait for empty buffer
	while(!( UCSR0A & (1<<UDRE0))){
		; //Do nothing
	}
	
	//Put data into the buffer, send the data
	UDR0 = data;
}

void serialPrint(char* string){
	unsigned int i =0;
	while (string[i]!='\0'){
		serialTransmitByte(string[i]);
		i++;
	}
	
}

void serialPrintLn(char* string){
	serialPrint(string);
	serialTransmitByte('\n');
}


char serialReceiveByte(){
	//Wait for data to be received
	while(!(UCSR0A & (1<<RXC0))){
		;//Do nothing
	}
	
	//Get and return the received byte
	return UDR0;
}

void serialEnableInterrupt(){
	
	UCSR0B |= 1<<RXCIE0;
	sei();
	
}

void serialSetRxBuffer(unsigned char* buff){
	serialRxBuffer = buff;
}

void serialSetRxBufferSize(unsigned int numBytes){
	serialRxBufferSize = numBytes;
}

void serialSetRxCompleteFlag(unsigned char* flag){
	serialRxCompleteFlag = flag;
}

ISR(USART_RX_vect){
	cli();
	unsigned char c = UDR0;
	if(c != '\n'){
		//Then it is data
		serialRxBuffer[serialNumBytesReceived] = c;
		serialNumBytesReceived++;
	}
	if(c == '\n'){
		serialRxBuffer[serialNumBytesReceived] = '\0';
		serialNumBytesReceived = 0;
		*(serialRxCompleteFlag) = 1; //Set the flag to 1 to say data received
	}
	sei();
}
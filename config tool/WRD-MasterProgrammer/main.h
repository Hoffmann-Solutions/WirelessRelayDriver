/*
 * main.h
 *
 * Created: 1/12/2019 9:32:53 AM
 *  Author: Cobus Hoffmann
 */ 


#ifndef MAIN_H_
#define MAIN_H_

//Define the pins needed for the spi interface
#define DDR_SPI DDRB
#define DD_MOSI 3
#define DD_MISO	4
#define DD_SCK	5
#define DD_SS	2
#define DD_SS2  1

#define NRF24L01_CE_PORT	PORTB
#define NRF24L01_CE_PIN		1
//Define some constants
#define OUTPUT				(uint8_t) 1
#define INPUT				(uint8_t) 0

#define OUTPUT_PORT			PORTD
#define OUTPUT_DDR			DDRD

//Define some macros to clear and set bits on a port
#define setbit(port, bit)		(port) |= (1<< (bit))
#define clearbit(port, bit)		(port) &= ~(1<<(bit))
//Define unsigned char as uint8_t


//Initialise the spi
void spi_init(void);
//Transmit a single char
void spi_transmit(char cData);
//Transmit and receive a number of bytes
void spi_transmit_receive(uint8_t *txBuff, uint8_t* buff, uint8_t numBytes);

//Enable and disable the nrf24l01 module
void nrf24l01_ce_low();			//Activate RX TX modes functions required by the nrf24l01 lib
void nrf24l01_ce_high();

void nrf24l01_csn_low();		//SPI Chip Select functions required by nrf24l01 lib
void nrf24l01_csn_high();

void wait_10us();				//Wait function required by the nrf24l01 lib

/**
 * \brief This function i used to querry if the message has been sent sucessfully or not
 * 
 * 
 * \return uint8_t 1:Sent 2:Max Retries 9:Unknown Error
 */
uint8_t nrf24l01MessageSent();




#endif /* MAIN_H_ */
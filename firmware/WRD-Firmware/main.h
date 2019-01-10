/*
 * main.h
 *
 * Created: 1/5/2019 4:09:53 PM
 *  Author: Cobus Hoffmann
 */ 


#ifndef MAIN_H_
#define MAIN_H_


//Default values for the EEPROM
#define INIT_PAIR_CODE	(uint8_t)0x00
#define INIT_MODE		(uint8_t)0x00
	
//Define the mode of operation here
#define _RX 1

//Will initially be undefined
#define MODE_PTX			(uint8_t)1
#define MODE_PRX			(uint8_t)2

//Used to distinguish if the message was for us
uint8_t EEMEM PIPE0_ADDR[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE8};  //EERPROM default conf needs to be loaded in seperate to .hex
uint8_t EEMEM PIPE1_ADDR[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC3};
uint8_t EEMEM MODE = MODE_PRX;


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
typedef unsigned char uint8_t;

//Initialise the spi
void spi_init(void);
//Transmit a single char
void spi_transmit(char cData);
//Transmit and receive a number of bytes
void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes);

//Enable and disable the nrf24l01 module
void nrf24l01_ce_low();			//Activate RX TX modes functions required by the nrf24l01 lib
void nrf24l01_ce_high();

void nrf24l01_csn_low();		//SPI Chip Select functions required by nrf24l01 lib
void nrf24l01_csn_high();

void wait_10us();				//Wait function required by the nrf24l01 lib

//Set the port c for input or output depending on the mode that the module is in
void setup_portd_gpio(uint8_t mode);

uint8_t read_portd_gpio();
void set_portd_gpio(uint8_t value);



void handlePTX(uint8_t *txBuff, uint8_t *rxBuff);

void handlePRX(uint8_t *rxBuff);




#endif /* MAIN_H_ */
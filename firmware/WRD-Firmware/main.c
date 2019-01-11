/*
 * WRD_v1.c
 *
 * Created: 11/9/2018 1:47:20 PM
 * Author : Cobus Hoffmann
 */ 

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "main.h"
#include "nrf24l01.h"


int status = 0;
uint8_t myMode = 0;
uint8_t pipe0Address[5] = {};
uint8_t pipe1Address[5] = {};
uint8_t currPipe = 0;


int main(void)
{
	spi_init();
	//Set the CE pin as output
	DDRB|=(1<<NRF24L01_CE_PIN);
	//Set it low to activate the nrf24l01 module
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	//Set the ss high
	setbit(PORTB, DD_SS);
	//Read the saved values
	myMode = eeprom_read_byte(&MODE);
	eeprom_read_block(pipe0Address, PIPE0_ADDR, sizeof(uint8_t)*5);
	eeprom_read_block(pipe1Address, PIPE1_ADDR, sizeof(uint8_t)*5);
	
	if (myMode == MODE_PTX){
		//Setup for TX
		nrf24l01SetNumRetries(3);
		nrf24l01_setup_tx();
		nrf24l01_reset_tx();
		setup_portd_gpio(INPUT);
	}else{
		//Setup for RX
		nrf24l01SetPayloadLen(pipe0, 3);
		nrf24l01SetPayloadLen(pipe1, 2);
		
		//set the address length for pipes
		nrf24l01SetPipeAddr(pipe0, pipe0Address, 5);
		nrf24l01SetPipeAddr(pipe1, pipe1Address, 5);
		//Power up the module
		nrf24l01_setup_rx();
		nrf24l01_reset_rx();
		//Setup the outputs 
		setup_portd_gpio(OUTPUT);
	}
	
	uint8_t sendData = 0;
	uint8_t txBuffMain[4];
	uint8_t rxBuffMain[4];
	
	clear(txBuffMain);
	clear(rxBuffMain);
	
    while (1) 
    {
		//Delay in the loop
		_delay_ms(100);
		switch(myMode){
			case MODE_PTX:
				handlePTX();
				break;
			case MODE_PRX:
				handlePRX();
				break;
		};	

    }
}


/**
 * @brief  This function initialises the spi
 * @param	None	
 * @retval	None
 */
void spi_init(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS)|(1<<DD_SS2);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

/**
 * @brief  This function is used to send data over spi
 * @param	char cData: the byte to be sent		
 * @retval None
 */
void spi_transmit(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;
}


/**
 * @brief  This function is used to send and receive data over spi
 * @param	uint8_t *txBuff:	Buffer for transmission
			uint8_t *rxBuffer:	Buffer for the received data
			uint8_t numBytes:	number of bytes to transmit and receive
			
 * @retval None
 */
void spi_transmit_receive(uint8_t* txBuff, uint8_t* buff, uint8_t numBytes){
	
	for(uint8_t i=0; i<numBytes; i++){
			/* Start transmission */
			SPDR = txBuff[i];
			/* Wait for transmission complete */
			while(!(SPSR & (1<<SPIF)))
			;
			buff[i]=SPDR;
	}
	
	
	
}


/**
 * @brief	This function is used to pull the chip enable pin low of the nrf24l01 module
 *			which allows the module to enter the active mode.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_ce_low(){
	
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	
}


/**
 * @brief	This function is used to pull the chip enable pin high of the nrf24l01 module
 *			which allows the module to enter the active mode.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_ce_high(){
	
	setbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
}



/**
 * @brief	This function is used to pull the spi chip selects pin low of the nrf24l01
 *			module which is needed for spi communication.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_csn_low(){
	PORTB &= ~(1<<DD_SS);
	
}

/**
 * @brief	This function is used to pull the spi chip selects pin high of the nrf24l01
 *			module which is needed for spi communication.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_csn_high(){
	PORTB |= (1<<DD_SS);
	
	
}

/**
 * @brief	This function is used to generate a delay greater than 10us needed by the nrf module
 *			to allow for a successful transmission.
 * @param	None
 *			
 * @retval	None
 */
void wait_10us(){
	_delay_us(10);
}

void setup_portd_gpio(uint8_t mode){
	switch (mode)
	{
		case INPUT:
		//clear the bits for input
		OUTPUT_DDR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4));
		
		break;
		case OUTPUT:
		//Set the bits for output
		OUTPUT_DDR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4);
		//Set the pins low
		PIND &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4));
		break;
	}
}

uint8_t read_portd_gpio(){
	uint8_t val = 0;
	//Read the lower 5 pins
	val = PIND & 0b00011111;
	return val;
}

void set_portd_gpio(uint8_t value){
	//Set the lower 5 bits of portc, ensure no data is entered for upper 3
	value = value&0x1F;	//Clear the top bits just in case
	PORTD = ((PORTD&0xE0)|(value));
	
}


void handlePTX(){
	char handlePRXBuff[4]={};
	uint8_t handlePTXBuff[4]={};
	
//Check the switches state
	//sendData = read_portd_gpio();
	//Set a dummy address
	handlePTXBuff[0]=0xaa;
	//Set the send address
	if(currPipe){
		nrf24l01SetTXAddr(pipe1Address, 5);
		nrf24l01SetPipeAddr(pipe1, pipe1Address, 5);
		currPipe=0;
		//Load the data to send
		handlePTXBuff[1]=0;
		nrf24l01_send_data(handlePTXBuff, 2);
	}else{
		nrf24l01SetTXAddr(pipe0Address, 5);
		nrf24l01SetPipeAddr(pipe0, pipe0Address, 5);
		currPipe=1;
		//Load the data
		handlePTXBuff[0]=0xAB;
		handlePTXBuff[1]=1;
		handlePTXBuff[2]=MODE_PTX;
		nrf24l01_send_data(handlePTXBuff, 3);
	}

	
	//Delay before checking
	_delay_ms(10);
	
	if(!(PINB&0x01)){
		//The IRQ pin is low
		nrf24l01_read_reg(0x07, handlePRXBuff, 1);
		if(0x01&(handlePRXBuff[0]>>5)){
			//Data sent successfully
			status = 1;
		}else if(0x01&(handlePRXBuff[0]>>4)){
			//Max retries
			status = 2;
		}
		//Reset the tx
		nrf24l01_reset_tx();
		}else{
		status = 9;
		nrf24l01_read_reg(0x07, handlePRXBuff, 1);
	}
	//*****************Extra delay for debugging
	_delay_ms(1000);
	
}

void handlePRX(){
	uint8_t nrf24l01Status = 0x00;
	uint8_t cmd = 0;
	uint8_t newAddr = 0;
	uint8_t newMode = 0;
	uint8_t relayValues =0;
	uint8_t handlePRXBuff[4]={};
	uint8_t handlePTXBuff[4]={};
		
	if(!(PINB&0x01)){
		//something happened
		nrf24l01Status = nrf24l01_get_status();
		nrf24l01_read_reg(0x07, handlePRXBuff, 1);
		if(0x01&(nrf24l01Status>>6)){
			//There be data to read
			
			//Check the pipe number, 0:config 1:data
			uint8_t dataPipeNum = 0x3&(nrf24l01Status>>1);
			switch(dataPipeNum){
				case 0:
					//Config pipe
					
					nrf24l01_read_rx(handlePRXBuff, 3);
					
					cmd = handlePRXBuff[0];
					newAddr = handlePRXBuff[1];

					if(handlePRXBuff[0] == CONFIG_CMD){
						//Config command received
						relayValues = handlePRXBuff[1];
						newMode = handlePRXBuff[2];
						//Update the EEPROM registers
						//pipe1Address[5] = newAddr;
						//myMode = newMode;
						//eeprom_write_block(pipe1Address, PIPE1_ADDR, 5);
						//eeprom_write_byte(&MODE, myMode);
						set_portd_gpio(relayValues);
					}else{
						//Not CMD
						status =99;
					}
					break;
				case 1:
					//Data pipe
					nrf24l01_read_rx(handlePRXBuff, 2);
					relayValues = handlePRXBuff[1];
					set_portd_gpio(relayValues);
					break;
				default:
					//No data, issue
					break;
				};
		}else{
			//I have no idea what happened	
		}
		//reset the rx
		nrf24l01_reset_rx();
	}
	
}


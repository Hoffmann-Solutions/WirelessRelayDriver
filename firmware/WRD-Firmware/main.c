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

int main(void)
{
	spi_init();
	//Set the CE pin as output
	DDRB|=(1<<NRF24L01_CE_PIN);
	//Set it low to activate the nrf24l01 module
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	//Set the ss high
	setbit(PORTB, DD_SS);
	
#ifdef _TX
	//Setup for TX mode
	nrf24l01_setup_tx();
	nrf24l01_reset_tx();
	//Set the switches as inputs
	setup_portd_gpio(INPUT);
#endif // _TX

#ifdef _RX
	//Setup for RX mode
	nrf24l01_setup_rx();
	nrf24l01_reset_rx();
	//Set the relays as outputs
	setup_portd_gpio(OUTPUT);
#endif // _RX

	
	uint8_t sendData = 0;
	uint8_t txBuff[4];
	uint8_t rxBuff[4];
	memset(txBuff, 0, sizeof(txBuff));
	memset(rxBuff, 0, sizeof(rxBuff));
	
	//Check the registers
	uint8_t data[10];
	memset(data, 0, sizeof(data));
	for(uint8_t i=0; i< 10; i++){
		nrf24l01_read_reg(i, data+i, 1);
	}
    while (1) 
    {
		//Delay in the loop
		_delay_ms(100);
		
#ifdef _TX
	//Chec the switches state
	sendData = read_portd_gpio();
	//Set a dummy address
	txBuff[0]=0xaa;
	//Load the data to send
	txBuff[1]=sendData;
	nrf24l01_send_data(txBuff, 2);
	//Delay before checking
	_delay_ms(10);
	
	if(!(PINB&0x01)){
		//The IRQ pin is low
		nrf24l01_read_reg(0x07, rxBuff, 1);
		if(0x01&(rxBuff[0]>>5)){
			//Data sent successfully
			status = 1;
			}else if(0x01&(rxBuff[0]>>4)){
			//Max retries
			status = 2;
		}
		//Reset the tx
		nrf24l01_reset_tx();
		}else{
		status = 9;
		nrf24l01_read_reg(0x07, rxBuff, 1);
	}
	
#endif // _TX
		
#ifdef _RX
	if(!(PINB&0x01)){
		//something happened
		nrf24l01_read_reg(0x07, rxBuff, 1);
		if(0x01&(rxBuff[0]>>6)){
			//There be data to read
			nrf24l01_read_rx(rxBuff, 2);
			//Set the outputs 
			uint8_t relayValues = rxBuff[1];
			set_portd_gpio(relayValues);
		}else{
		//I have no idea what happened
			
		}
		//reset the rx
		nrf24l01_reset_rx();
	}
#endif // _RX

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
void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes){
	
	//Clear the receive buffer
	memset(rxBuff, '\0', sizeof(rxBuff));
	
	for(uint8_t i=0; i<numBytes; i++){
			/* Start transmission */
			SPDR = txBuff[i];
			/* Wait for transmission complete */
			while(!(SPSR & (1<<SPIF)))
			;
			rxBuff[i]=SPDR;
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
		OUTPUT_DDR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
		
		break;
		case OUTPUT:
		//Set the bits for output
		OUTPUT_DDR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
		//Set the pins low
		PIND &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
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


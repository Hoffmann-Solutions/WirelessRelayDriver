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
#include "main.h"
#include "serial.h"
#include "nrf24l01.h"

const char CHECK_OK_CMD[] = "AT";
const char SET_ADDR_CMD[] = "AT+SA:";
const char SET_MODE_CMD[] = "AT+SM:";
const char SEND_CMD[] = "AT+SND:";
const char QRY_CMD[] = "AT+?";

int myAddr = 0;
int myMode = 0;

//Default pipe0 addr programmed into all WRDs
uint8_t PIPE0_ADDR[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE8};	

int main(void)
{
	//Enable the spi interface
	spi_init();
	//Set the CE pin as output
	DDRB|=(1<<NRF24L01_CE_PIN);
	//set it low to activate the nrf24l01
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	//Set the ss high
	setbit(PORTB, DD_SS);
	
	//Set the UART interface
	serialInit(MYUBRR);
	char myBuffer[20];
	volatile char myFlag = 0;
	
	memset(myBuffer, '\0', sizeof(myBuffer));
	//Set buffer and flag for the serial stuff
	serialSetRxBuffer(myBuffer);
	serialSetRxCompleteFlag(&myFlag);
	//Enable the reception via interrupt
	serialEnableInterrupt();
	
	//Set the nrf24l01 module to TX mode
	nrf24l01SetNumRetries(3);
	//Set the default pipe0 and TX address
	nrf24l01SetPipeAddr(pipe0, PIPE0_ADDR, 5);
	nrf24l01SetTXAddr(PIPE0_ADDR, 5);
	nrf24l01_setup_tx();
	nrf24l01_reset_rx();
	
	//Create the transmit buffer
	uint8_t txBuffer[5];
	memset(txBuffer, 0, sizeof(txBuffer));
	txBuffer[0]=0xAB;
	
    while (1) 
    {
		if (myFlag)
		{
			//Serial data has been received
			serialPrint("> ");
			serialPrintLn(myBuffer);
			myFlag = 0;
			//Check if the command received matches SET_ADDR_CMD
			if(!strncmp(SET_ADDR_CMD, myBuffer, strlen(SET_ADDR_CMD))){
				//Address command received 
				myAddr = atoi(myBuffer+strlen(SET_ADDR_CMD));
				txBuffer[1]=(uint8_t)myAddr;
				
			}else if(!strncmp(SET_MODE_CMD, myBuffer, strlen(SET_MODE_CMD))){
				//Mode command received
				myMode = atoi(myBuffer+strlen(SET_MODE_CMD));
				txBuffer[2] = (uint8_t)myMode;
				
			}else if(!strncmp(QRY_CMD, myBuffer, strlen(QRY_CMD))){
				//Check the current loaded
				serialPrint("ADDR: ");
				char str[10];
				memset(str, '\0', sizeof(str));
				itoa(myAddr, str, 10);
				serialPrintLn(str);
				serialPrint("MODE: ");
				memset(str, '\0', sizeof(str));
				itoa(myMode, str, 10);
				serialPrintLn(str);
				
			}else if(!strncmp(CHECK_OK_CMD, myBuffer, strlen(CHECK_OK_CMD))){
			//Check command
			serialPrintLn("OK");
		}
			
		}
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







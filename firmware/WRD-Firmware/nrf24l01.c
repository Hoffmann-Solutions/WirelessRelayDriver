#include "nrf24l01.h"


/****************Commands*****************/
#define R_MASK							(uint8_t)0b00000000
#define W_MASK							(uint8_t)0b00100000
#define R_RX_PAYLOAD					(uint8_t)0b01100001
#define W_TX_PAYLOAD					(uint8_t)0b10100000
#define FLUSH_TX						(uint8_t)0b11100001
#define FLUSH_RX						(uint8_t)0b11100010
#define NOP 							(uint8_t)0xff			

/***********Registers********************/
//Config register
#define CONFIG_REG					0x00
/*
 * 7: 	Reserved
 * 6:	MASK_RX_DR
 * 5:	MASK_TX_DS
 * 4:	MASK_MAX_RT
 * 3:	EN_CRC
 * 2:	CRCO
 * 1:	PWR_UP						[1: PWR UP, 0: PWR DOWN]
 * 0:	PRIM_RX						RX/TX Control [1: PRX, 0: PTX]
 */
#define	EN_CRC						(uint8_t)3
#define	PWR_UP						(uint8_t)1
#define PRIM_RX						(uint8_t)0

#define EN_AA_REG 					(uint8_t)0x01
#define EN_RXADDR_REG				(uint8_t)0x02
#define ENRX_P5						(uint8_t)5
#define ENRX_P4						(uint8_t)4
#define ENRX_P3						(uint8_t)3
#define ENRX_P2						(uint8_t)2
#define ENRX_P1						(uint8_t)1
#define ENRX_P0						(uint8_t)0
//Address to set the Address widths
#define SETUP_AW_REG				(uint8_t)0x03
#define AW_1						(uint8_t)1
#define AW_0						(uint8_t)0

#define SETUP_RETR_REG				(uint8_t)0x04
#define RF_CH_REG					(uint8_t)0x05
#define RF_SETUP_REG				(uint8_t)0x06
//Status register
#define STATUS_REG					(uint8_t)(0x07)

/*
 * 7: 	Reserved
 * 6:	RX_DR
 * 5:	TX_DS						Set when data successfully sent
 * 4:	MAX_RT						Maximum retransmit tries, write 1 to clear
 * 3:	RX_P_NO[3]
 * 2:	RX_P_NO[2]
 * 1:	RX_P_NO[1]					[1: PWR UP, 0: PWR DOWN]
 * 0:	TX_FULL						TX_FIFO full
 */
#define RX_DR						(uint8_t)6
#define TX_DS						(uint8_t)5
#define MAX_RT						(uint8_t)4
#define TX_FULL						(uint8_t)0
//Address registers
#define RX_ADDR_P0					(uint8_t)0x0A
#define RX_ADDR_P1					(uint8_t)0x0B
#define RX_ADDR_P2					(uint8_t)0x0C
#define RX_ADDR_P3					(uint8_t)0x0D
#define RX_ADDR_P4					(uint8_t)0x0E
#define RX_ADDR_P5					(uint8_t)0x0F

#define RX_PW_P0_REG				(uint8_t)0x11

//Local buffers used for the tx and rx transactions
uint8_t txBuff[TX_RX_BUFF_LEN];
uint8_t rxBuff[TX_RX_BUFF_LEN];


/************************************************************************/
/* Initialise the nrf24l01 module and ensure that the spi is initialised*/
/************************************************************************/

void nrf24l01_setup_tx(void){
	//Clear the buffers
	clear(txBuff);
	clear(rxBuff);

	//Make sure that the moduule is not active
	nrf24l01_ce_low();

	//Enable the RX address for pipe 0
	txBuff[0]=(W_MASK)|(EN_RXADDR_REG);
	txBuff[1]=(1<<0);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();


	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Set the write mask and then set number of payload len
	txBuff[0]=(W_MASK|RX_PW_P0_REG);
	txBuff[1]=(NRF24L01_PAYLOAD_LEN);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();

	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Set the write mask and then set number of payload len
	txBuff[0]=(W_MASK|SETUP_RETR_REG);
	txBuff[1]=(0x13);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();


	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Power up the module
	txBuff[0]=(W_MASK)|(CONFIG_REG);
	txBuff[1]=(1<<EN_CRC)|(1<<PWR_UP);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();

}


void nrf24l01_setup_rx(void){
	//Clear the buffers
	clear(txBuff);
	clear(rxBuff);

	//Make sure that the module is not active
	nrf24l01_ce_low();

	//Check that the registers where set correct
	clear(txBuff);
	clear(rxBuff);

	//Set the write mask and then set number of payload len
	txBuff[0]=(W_MASK|RX_PW_P0_REG);
	txBuff[1]=(NRF24L01_PAYLOAD_LEN);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();

	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Set the write mask and then set number of payload len
	txBuff[0]=(W_MASK|EN_RXADDR_REG);
	txBuff[1]=(1<<0);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();


	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Set the registers
	txBuff[0]	=		(W_MASK|CONFIG_REG);
	txBuff[1]		=	(1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();


	//Set the module in active RX mode
	nrf24l01_ce_high();

}


void nrf24l01_send_data(uint8_t *txData, uint8_t numBytes){

	//clear the buffers
	clear(txBuff);
	clear(rxBuff);

	//Clear the buffers
	clear(txBuff);
	clear(rxBuff);

	//copy the new content into the txBuff
	txBuff[0]=(W_TX_PAYLOAD);
	memcpy(txBuff+sizeof(uint8_t), txData, numBytes);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, numBytes+1);
	nrf24l01_csn_high();

	//Now pulse the CE pin for at least 10 us but no more than 4ms
	nrf24l01_ce_high();
	wait_10us();
	nrf24l01_ce_low();

}

uint8_t nrf24l01_get_status(){
	uint8_t send = NOP;
	uint8_t receive = 0x00;
	nrf24l01_csn_low();
	spi_transmit_receive(&send, &receive, 1);
	nrf24l01_csn_high();

	return receive;

}

uint8_t nrf24l01_get_config(){
	uint8_t send[] = {R_MASK|CONFIG_REG, NOP};
	uint8_t receive[]={0,0x00};

	nrf24l01_csn_low();
	spi_transmit_receive(send, receive, 2);
	nrf24l01_csn_high();

	return receive[1];

}

uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t *data, uint8_t numBytes){

	clear(rxBuff);
	clear(txBuff);

	txBuff[0]=(W_MASK)|(reg);
	memcpy(txBuff+sizeof(uint8_t), data, numBytes);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, numBytes+1);
	nrf24l01_csn_high();


}

uint8_t nrf24l01_read_reg(uint8_t reg, uint8_t *buff, uint8_t numBytes){
	clear(rxBuff);
	clear(txBuff);

	txBuff[0]=(R_MASK)|(reg);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, numBytes+1);
	nrf24l01_csn_high();

	//Copy data into the buffer supplied
	memcpy(buff, rxBuff+sizeof(uint8_t), numBytes);


}


uint8_t nrf24l01_read_rx(uint8_t *buff, uint8_t numBytes){
	clear(rxBuff);
	clear(txBuff);

	txBuff[0]=(R_RX_PAYLOAD);


	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, numBytes+1);
	nrf24l01_csn_high();

	memcpy(buff, rxBuff+sizeof(uint8_t), numBytes);

}

void nrf24l01_reset_tx(void){
	//Use this function when max_rt or TX_DS gets triggered
	clear(txBuff);
	clear(rxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	txBuff[0] = (W_MASK)|(STATUS_REG);
	txBuff[1]=(1<<TX_DS)|(1<<MAX_RT);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();

	clear(txBuff);
	clear(rxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	txBuff[0] = (FLUSH_TX);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 1);
	nrf24l01_csn_high();

}

void nrf24l01_reset_rx(void){
	//Use this after data received

	//Reset the RX_DR
	clear(txBuff);
	clear(rxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	txBuff[0] = (W_MASK)|(STATUS_REG);
	txBuff[1]=(1<<RX_DR);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();


	clear(txBuff);
	clear(rxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	txBuff[0] = (FLUSH_RX);

	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 1);
	nrf24l01_csn_high();

}

void nrf24l01EnablePipe(PipeNum_t pipeNum){
	clear(txBuff);
	clear(rxBuff);
	
	//Set up the config data to send
	txBuff[0] = (W_MASK)|(EN_RXADDR_REG);
	switch(pipeNum){
		case pipe0:
			txBuff[1]=1<<ENRX_P0;
		break;
		case pipe1:
			txBuff[1]=1<<ENRX_P1;
		break;
		case pipe2:
			txBuff[1]=1<<ENRX_P2;
		break;
		case pipe3:
			txBuff[1]=1<<ENRX_P3;
		break;
		case pipe4:
			txBuff[1]=1<<ENRX_P4;
		break;
		case pipe5:
			txBuff[1]=1<<ENRX_P5;
		break;
	};
	
	//Transmit the config data
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();

}

void nrf24l01SetPipeAddr(PipeNum_t pipeNum, uint8_t addr){
	clear(txBuff);
	clear(rxBuff);
	int addrLen = 0;
	
	switch(pipeNum){
		case pipe0:
			txBuff[0] = (W_MASK)|(RX_ADDR_P0);
			addrLen = 5;
			break;
		case pipe1:
			txBuff[0] = (W_MASK)|(RX_ADDR_P1);
			addrLen = 5;
		break;
		case pipe2:
			txBuff[0] = (W_MASK)|(RX_ADDR_P2);
			addrLen = 1;
		break;
		case pipe3:
			txBuff[0] = (W_MASK)|(RX_ADDR_P3);
			addrLen = 1;
		break;
		case pipe4:
			txBuff[0] = (W_MASK)|(RX_ADDR_P4);
			addrLen = 1;
		break;
		case pipe5:
			txBuff[0] = (W_MASK)|(RX_ADDR_P5);
			addrLen = 1;
		break;
	};
	
	//load in the addr
	txBuff[1] = addr;
	
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 1+addrLen);
	nrf24l01_csn_high();
	
}

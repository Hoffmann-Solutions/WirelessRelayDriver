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

#define TX_ADDR_REG					(uint8_t)0x10

#define RX_PW_P0_REG				(uint8_t)0x11
#define RX_PW_P1_REG				(uint8_t)0x12
#define RX_PW_P2_REG				(uint8_t)0x13
#define RX_PW_P3_REG				(uint8_t)0x14
#define RX_PW_P4_REG				(uint8_t)0x15
#define RX_PW_P5_REG				(uint8_t)0x16


//Local buffers used for the tx and rx transactions
uint8_t nrf24l01TxBuff[TX_RX_BUFF_LEN];
uint8_t nrf24l01RxBuff[TX_RX_BUFF_LEN];


/************************************************************************/
/* Initialise the nrf24l01 module and ensure that the spi is initialised*/
/************************************************************************/

void nrf24l01_setup_tx(void){
	//Clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);

	//Make sure that the moduule is not active
	nrf24l01_ce_low();

	//Enable the RX address for pipe 0
// 	txBuff[0]=(W_MASK)|(EN_RXADDR_REG);
// 	txBuff[1]=(1<<0);
// 	nrf24l01_csn_low();
// 	spi_transmit_receive(txBuff, rxBuff, 2);
// 	nrf24l01_csn_high();


	//clear the buffers
// 	clear(txBuff);
// 	clear(rxBuff);
// 	//Set the write mask and then set number of payload len
// 	txBuff[0]=(W_MASK|RX_PW_P0_REG);
// 	txBuff[1]=(NRF24L01_PAYLOAD_LEN);
// 	nrf24l01_csn_low();
// 	spi_transmit_receive(txBuff, rxBuff, 2);
// 	nrf24l01_csn_high();

	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Set the write mask and then set the number of retries
// 	txBuff[0]=(W_MASK|SETUP_RETR_REG);
// 	txBuff[1]=(0x13);
// 	nrf24l01_csn_low();
// 	spi_transmit_receive(txBuff, rxBuff, 2);
// 	nrf24l01_csn_high();


	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Power up the module
	nrf24l01TxBuff[0]=(W_MASK)|(CONFIG_REG);
	nrf24l01TxBuff[1]=(1<<EN_CRC)|(1<<PWR_UP);
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();

}


void nrf24l01_setup_rx(void){
	//Clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);

	//Make sure that the module is not active
	nrf24l01_ce_low();

	//Check that the registers where set correct
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);

	//Set the write mask and then set number of payload len
// 	txBuff[0]=(W_MASK|RX_PW_P0_REG);
// 	txBuff[1]=(NRF24L01_PAYLOAD_LEN);
// 	nrf24l01_csn_low();
// 	spi_transmit_receive(txBuff, rxBuff, 2);
// 	nrf24l01_csn_high();

	//clear the buffers
// 	clear(txBuff);
// 	clear(rxBuff);
// 	//Set the write mask and then set the pipe
// 	txBuff[0]=(W_MASK|EN_RXADDR_REG);
// 	txBuff[1]=(1<<0);
// 	nrf24l01_csn_low();
// 	spi_transmit_receive(txBuff, rxBuff, 2);
// 	nrf24l01_csn_high();


	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Set the registers
	nrf24l01TxBuff[0]	=		(W_MASK|CONFIG_REG);
	nrf24l01TxBuff[1]		=	(1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();


	//Set the module in active RX mode
	nrf24l01_ce_high();

}


void nrf24l01_send_data(uint8_t *txData, uint8_t numBytes){

	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);

	//Clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);

	//copy the new content into the txBuff
	nrf24l01TxBuff[0]=(W_TX_PAYLOAD);
	memcpy(nrf24l01TxBuff+sizeof(uint8_t), txData, numBytes);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, numBytes+1);
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

	clear(nrf24l01RxBuff);
	clear(nrf24l01TxBuff);

	nrf24l01TxBuff[0]=(W_MASK)|(reg);
	memcpy(nrf24l01TxBuff+sizeof(uint8_t), data, numBytes);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, numBytes+1);
	nrf24l01_csn_high();


}

uint8_t nrf24l01_read_reg(uint8_t reg, uint8_t *buff, uint8_t numBytes){

	nrf24l01TxBuff[0]=(R_MASK)|(reg);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, numBytes+1);
	nrf24l01_csn_high();

	//Copy data into the buffer supplied
	memcpy(buff, nrf24l01RxBuff+1, numBytes);


}


uint8_t nrf24l01_read_rx(char *buff, uint8_t numBytes){

	nrf24l01TxBuff[0]=(R_RX_PAYLOAD);


	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, numBytes+1);
	nrf24l01_csn_high();

	memcpy(buff, nrf24l01RxBuff+1, numBytes);

}

void nrf24l01_reset_tx(void){
	//Use this function when max_rt or TX_DS gets triggered
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	nrf24l01TxBuff[0] = (W_MASK)|(STATUS_REG);
	nrf24l01TxBuff[1]=(1<<TX_DS)|(1<<MAX_RT);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();

	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	nrf24l01TxBuff[0] = (FLUSH_TX);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 1);
	nrf24l01_csn_high();

}

void nrf24l01_reset_rx(void){
	//Use this after data received

	//Reset the RX_DR
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Flush the TX register


	//Reset the MRX_DR
	nrf24l01TxBuff[0] = (W_MASK)|(STATUS_REG);
	nrf24l01TxBuff[1]=(1<<RX_DR);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();


	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Flush the TX register


	//Reset the MAX_RT and TX_DS flag
	nrf24l01TxBuff[0] = (FLUSH_RX);

	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 1);
	nrf24l01_csn_high();

}


void nrf24l01EnablePipe(PipeNum_t pipeNum){
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	
	//Set up the config data to send
	nrf24l01TxBuff[0] = (W_MASK)|(EN_RXADDR_REG);
	switch(pipeNum){
		case pipe0:
			nrf24l01TxBuff[1]=1<<ENRX_P0;
		break;
		case pipe1:
			nrf24l01TxBuff[1]=1<<ENRX_P1;
		break;
		case pipe2:
			nrf24l01TxBuff[1]=1<<ENRX_P2;
		break;
		case pipe3:
			nrf24l01TxBuff[1]=1<<ENRX_P3;
		break;
		case pipe4:
			nrf24l01TxBuff[1]=1<<ENRX_P4;
		break;
		case pipe5:
			nrf24l01TxBuff[1]=1<<ENRX_P5;
		break;
	};
	
	//Transmit the config data
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();

}

void nrf24l01SetPipeAddr(PipeNum_t pipeNum, uint8_t *addr, uint8_t numBytes){
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	
	switch(pipeNum){
		case pipe0:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P0);
			break;
		case pipe1:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P1);
		break;
		case pipe2:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P2);
		break;
		case pipe3:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P3);
		break;
		case pipe4:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P4);
		break;
		case pipe5:
			nrf24l01TxBuff[0] = (W_MASK)|(RX_ADDR_P5);
		break;
	};
	
	//copy the addr to the txBuffer
	memcpy(nrf24l01TxBuff+1, addr, numBytes);
	
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 1+numBytes);
	nrf24l01_csn_high();
	
}


void nrf24l01SetTXAddr(uint8_t *addr, uint8_t numBytes){
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Set the write command and TX_ADDR_REG
	nrf24l01TxBuff[0] = (W_MASK)|(TX_ADDR_REG);
	//Coppy the address into txBuff
	memcpy(nrf24l01TxBuff+1, addr, numBytes);
	
	//Set the addr
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 1+numBytes);
	nrf24l01_csn_high();
	
}


void nrf24l01SetPayloadLen(PipeNum_t pipeNum, uint8_t numBytes){
	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Set the write mask and then set number of payload len
	switch (pipeNum)
	{
	case pipe0:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P0_REG);
		break;
	case pipe1:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P1_REG);
		break;
	case pipe2:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P2_REG);
		break;
	case pipe3:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P3_REG);
		break;
	case pipe4:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P4_REG);
		break;
	case pipe5:
		nrf24l01TxBuff[0]=(W_MASK|RX_PW_P5_REG);
		break;
	}
	
	nrf24l01TxBuff[1]=(numBytes);
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();
}

void nrf24l01SetNumRetries(uint8_t numRetries){
	//clear the buffers
	clear(nrf24l01TxBuff);
	clear(nrf24l01RxBuff);
	//Set the write mask and then set the number of retries
	nrf24l01TxBuff[0]=(W_MASK|SETUP_RETR_REG);
	nrf24l01TxBuff[1]=(1<<4)|(numRetries);//500us delay between retries
	nrf24l01_csn_low();
	spi_transmit_receive(nrf24l01TxBuff, nrf24l01RxBuff, 2);
	nrf24l01_csn_high();
}
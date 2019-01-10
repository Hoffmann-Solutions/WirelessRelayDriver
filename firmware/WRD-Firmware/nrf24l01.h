#ifndef NRF24L01_H
#define NRF24L01_H

#include <string.h>
#include <avr/io.h>

//Enum typedef to store the available pipes
typedef enum pipeType{pipe0, pipe1, pipe2, pipe3, pipe4, pipe5} PipeNum_t;

#define clear(x)	memset((x), 0, sizeof((x)))

/******************Defines**************************/
#define TX_RX_BUFF_LEN					(uint8_t)20
#define NRF24L01_PAYLOAD_LEN			(uint8_t)2


/*****************External functions ***************/
extern void nrf24l01_csn_low();			//SPI Chip Select
extern void nrf24l01_csn_high();
extern void nrf24l01_ce_low();			//Chip enable activates RX or TX mode
extern void nrf24l01_ce_high();
extern void wait_10us();

extern void spi_init();
extern void spi_transmit(char cData);
extern void spi_transmit_receive(uint8_t *txBuff, char *buff, uint8_t numBytes);

/*******************Includes************************/

/*******************Functions***********************/

/**
 * @brief  This function initialises the nrf24l01 modules
 * @param	None
 * @retval	None
 */
void nrf24l01_init(void);

/**
 * @brief  This function sets up the nrf24l01 ic in transmit mode
 * @param	None
 * @retval	None
 */
void nrf24l01_setup_tx(void);

/**
 * @brief  This function sets up the nrf24l01 ic in receive mode
 * @param	None
 * @retval	None
 */
void nrf24l01_setup_rx(void);

/**
 * @brief  This function sends data to another module, it is used when in TX mode
 * @param	uint8_t *txData:	pointer to the transmit buffer
			uint8_t numBytes:	number of bytes to be sent
 * @retval	None
 */
void nrf24l01_send_data(uint8_t *txData, uint8_t numBytes);

uint8_t nrf24l01_get_status();

uint8_t nrf24l01_get_config();

/**
 * @brief	This function writes data to the internal registers of the nrf24l01 ic
 * @param	uint8_t reg:		The register to be manipulated
			uint8_t *data:		Data buffer to be written to the register
			uint8_t numBytes:	The number of bytes to be sent
 * @retval	None
 */
uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t *data, uint8_t numBytes);

/**
 * @brief	This function is used to read the internal registers of the nrf24l01 ic
 * @param	None
 * @retval	None
 */
uint8_t nrf24l01_read_reg(uint8_t reg, uint8_t *buff, uint8_t numBytes);

/**
 * @brief	This function is used to read the RX buffer, used when the ic is in rx mode
 * @param	uint8_t *buff:		Buffer to store the received data
			uint8_t numBytes:	Number of bytes to read
 * @retval	None
 */
uint8_t nrf24l01_read_rx(char *buff, uint8_t numBytes);

/**
 * @brief	This function is used to reset IRQ flags and to clear the TX buffer, used when a MAX_RT error
			occurs.
 * @param	None
 * @retval	None
 */
void nrf24l01_reset_tx(void);

/**
 * @brief	This function is used to reset the IRQ flags and to clear the RX buffer, used after reading
			the RX buffer to ensure module is ready to receive more data.
 * @param	None
 * @retval	None
 */
void nrf24l01_reset_rx(void);

/**
 * @brief This function is used to enable one of the 6 available data pipes of the nrf24l01
 *
 * @param PipeNum_t 
 * @retval None
 */
void nrf24l01EnablePipe(PipeNum_t pipeNum);


/**
 * @brief This function is used to set the pipe address
 *
 * @param PipeNum_t 
 * @param PipeNum_t
 * @retval None
 */
void nrf24l01SetPipeAddr(PipeNum_t pipeNum, uint8_t *addr, uint8_t numBytes);

/**
 * \brief This method is used to set the transmitting addr
 *	used to set the pipe address to transmit to
 * \param addr
 * 
 * \return void
 */
void nrf24l01SetTXAddr(uint8_t *addr, uint8_t numBytes);

/**
 * \brief This method is used to set the payload length
 * 
 * \param numBytes
 * 
 * \return void
 */
void nrf24l01SetPayloadLen(PipeNum_t pipeNum, uint8_t numBytes);


/**
 * \brief This method is used to set the number of retries for
 * when in transmitter mode.
 * 
 * \param numRetries
 * 
 * \return void
 */
void nrf24l01SetNumRetries(uint8_t numRetries);

#endif

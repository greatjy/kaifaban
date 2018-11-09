#ifndef COM_H
#define COM_H
/*============================ INCLDUE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include <clock_config.h>
#include "compiler.h"
/*============================ MACROS ========================================*/
#define ENABLE_RECEIVER  ( UCSR0B |= ( 1 << RXEN0 ) ) /*!< Enables receiver. */
#define DISABLE_RECEIVER ( UCSR0B &= ~( 1 << RXEN0 ) ) /*!< Disables receiver. */

#define ENABLE_TRANSMITTER  ( UCSR0B |= ( 1 << TXEN0 ) ) /*!< Enables transmitter. */
#define DISABLE_TRANSMITTER ( UCSR0B &= ~( 1 << TXEN0 ) ) /*!< Disables transmitter. */

#define ENABLE_RECEIVE_COMPLETE_INTERRUPT  ( UCSR0B |= ( 1 << RXCIE0 ) ) /*!< Enables an interrupt each time the receiver completes a symbol. */
#define DISABLE_RECEIVE_COMPLETE_INTERRUPT ( UCSR0B &= ~( 1 << RXCIE0 ) ) /*!< Disables an interrupt each time the receiver completes a symbol. */

#define LOW ( 0x00 )
#define XRAM_ENABLE( )     XMCRA |= ( 1 << SRE ); XMCRB |= ( 1 << XMBK )
#define XRAM_DISABLE( )    XMCRA &= ~( 1 << SRE )

#define FTDI_PORT        		( PORTE )
#define FTDI_DDR         		( DDRE )
#define FTDI_PIN         		( PINE )
#define FTDI_TX_PIN		 		( PINE6 ) //Transmit buffer empty. PE6
#define	FTDI_TX_MASK	 		( 1 << FTDI_TX_PIN )
#define	FTDI_RX_PIN		 		( PINE7 ) //Receive buffer full. PE7
#define FTDI_FIFO_ADDRESS     	( 0xF000 )
#define FTDI_Fifo ( ( volatile uint8_t * )FTDI_FIFO_ADDRESS )

#define FTDI_ENABLE_TX( )						( FTDI_DDR &= ~( 1 << FTDI_TX_PIN ) )
#define FTDI_CONFIGURE_PIN_CHANGE_INTERRUPT( )	( EICRB &= ~( ( 1 << ISC71 ) | ( 1 << ISC70 ) ) )
#define FTDI_ENABLE_RECEIVER( )					( EIMSK |= ( 1 << FTDI_RX_PIN ) )
#define FTDI_DISABLE_RECEIVER( )				(EIMSK &= ~( 1 << FTDI_RX_PIN ) )
/*============================ TYPEDEFS ======================================*/
/*! \brief  Enumeration that defines the available baud rates for\n
 *          the serial interface. The values cana be found in the
 *          datasheet on page 233.
 *  
 *  \note The members should not be altered. This is possibly harmful for\n
 *        setting of the baud rate registers in serialInterfaceInitialization.
 */
#if ( F_CPU == 7372800UL )
typedef enum
{//Main Freq= 7.3728MHz
	BR_2400= 191,
	BR_4800= 95,
	BR_9600  = 47, /*!< Sets the baud rate to 9600. */
	BR_14400 = 31,
	BR_19200 = 23, /*!< Sets the baud rate to 19200. */
	BR_28800 = 15,
	BR_31250 = 13,
	BR_38400 = 11,  /*!< Sets the baud rate to 38400. */
	BR_57600 = 7,
	BR_115200= 3
}baud_rate_t;
#elif ( F_CPU == 8000000UL )
typedef enum
{//Main Freq= 8.0 MHz
	BR_2400= 207,
	BR_4800= 103,
	BR_9600  = 51, /*!< Sets the baud rate to 9600. */
	BR_14400 = 33,
	BR_19200 = 25, /*!< Sets the baud rate to 19200. */
	BR_28800 = 16,
	BR_31250 = 15,
	BR_38400 = 12,  /*!< Sets the baud rate to 38400. */
	BR_57600 = 7,
	BR_115200= 3
}baud_rate_t;
#else
    #error "Clock speed not supported."
#endif/*============================ VARIABLES =====================================*/
/*============================ PROTOTYPES ====================================*/

void com_init( baud_rate_t rate );
void com_send_string( uint8_t *data, uint8_t data_length );
void com_send_hex( uint8_t nmbr );
uint8_t * com_get_received_data( void );
uint8_t com_get_number_of_received_bytes( void );
void com_reset_receiver( void );
#endif

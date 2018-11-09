/*============================ INCLDUE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include <clock_config.h>
#include "config_uart_extended.h"

#include "compiler.h"
#include "com.h"
/*============================ MACROS ========================================*/
#define COM_RX_MAX_BYTES (COM_RX_BUFFER_SIZE - 2) //!< Maximal number of bytes that one message can contain (PSDU_LENGTH - CRC_LENGTH).
/*============================ TYPEDEFS ======================================*/
/*============================ VARIABLES =====================================*/
//static uint8_t com_buffer[ COM_RX_BUFFER_SIZE ]; //!< Array storing rx data.
static uint8_t com_buffer0[ COM_RX_BUFFER_SIZE ]; //!< Array storing rx data.
static uint8_t com_buffer1[ COM_RX_BUFFER_SIZE ];
static uint8_t * com_buffer=com_buffer0;
static bool ori_buf_flag=false;	// false->buf0 true->buf1

static uint8_t com_number_of_received_bytes,com_number_of_received_bytesA,com_number_of_received_bytesB; //!< Number of bytes in com_buffer.
static bool com_data_reception_finished; //!< Flag indicating EOT (That "\r\n is received.")
static uint8_t hex_lookup[ ] = "0123456789ABCDEF"; //!< Look up table for hexadecimal number conversion.
static uint8_t dec_lookup[] = "0123456789";
/*============================ PROTOTYPES ====================================*/

/*! \brief This function initializes the chosen communication interface (USB or USART).
 *  
 *  \param[in] rate Baudrate used by the AVR's USART.
 */
void com_init( uint8_t rate ){
  

    UBRR0H = 0x00;
    UBRR0L = rate;
  
    //Enable USART transmitter module. Always on.
    ENABLE_RECEIVER;
    ENABLE_TRANSMITTER;
	
    //8-N-1.
    UCSR0C |= ( 1 << UCSZ01 ) | ( 1 << UCSZ00 ); 
  
    com_number_of_received_bytes = 0;
    com_data_reception_finished = false;
    ENABLE_RECEIVE_COMPLETE_INTERRUPT;

}

/*! \brief This function sends data on the chosen communication interface (USB or USART).
 *  
 *  \param[in] data Pointer to data that is to be sent on the communication interface.
 *  \param[in] data_length Number of bytes to read from the array pointed to by data.
 */
void com_send_string( uint8_t *data, uint8_t data_length ){
    
    while (--data_length > 0) {
        for(; !(UCSR0A & (1 << UDRE0));) {;}
  	    UDR0 = *data++; //Put symbol in data register.
    
    }    
}

/*! \brief This function prints the supplied argument as a hex number.
 *  
 *  \param[in] nmbr Number to be printed as a hexadescimal number.
 */
 void com_send_hex( uint8_t nmbr ){
	

			//for(; !(UCSR0A & (1 << UDRE0));) {;}
			//UDR0 = '0'; //Put symbol in data register.
		
			//for(; !(UCSR0A & (1 << UDRE0));) {;}
			//UDR0 = 'x'; //Put symbol in data register.
			
			for(; !(UCSR0A & (1 << UDRE0));) {;}
			UDR0 = hex_lookup[ ( nmbr >> 4 ) & 0x0F ];
			
			for(; !(UCSR0A & (1 << UDRE0));) {;}
			UDR0 = hex_lookup[ ( nmbr & 0x0F ) ];
	}
	

/*! \brief This function retruns the address to the first byte in the buffer where received data is stored3.
 *  
 *  \note This function should only be called after it has been verified that the 
 *        data reception is done (com_get_number_of_received_bytes() != 0).
 *  \return Pointer to the first byte in the array of received data.
 */
/*uint8_t * com_get_received_data( void ){
    return &com_buffer[0];
}*/

uint8_t * com_get_received_data( void ){

	if(ori_buf_flag){
		return com_buffer0;
	}else{
		return com_buffer1;
	}
}


/*! \brief This function returns number of bytes received during last data reception.
 *  
 *  \retval 0 No data is available. Data reception is not done.
 *  \retval 1 Error: Typed Frame Too Long.
 *  \return Any non zero value returned indicatest that data is available and should be read.
 */
uint8_t com_get_number_of_received_bytes( void ){
    
    if (com_data_reception_finished == true) {
		if(ori_buf_flag)
        		return com_number_of_received_bytesA; 
		else
			return com_number_of_received_bytesB; 
    } else { return 0; }
}

/*! \brief This function is used to reset the commuincation interface after each 
 *         data reception is done, and the end-user has read data.
 *  
 */
void com_reset_receiver( void ){
    
    DISABLE_RECEIVE_COMPLETE_INTERRUPT;
    
//    com_number_of_received_bytes = 0;
    com_data_reception_finished = false;
    
    uint8_t dummy = 0;
    //Following loop is used to ensure that the rx FIFO is flushed.
	//Sometimes it gets cloged up with old data.
	for( ;  UCSR0A & ( 1 << RXC0 ); )
	{
		dummy = UDR0;  
	}
    
    ENABLE_RECEIVE_COMPLETE_INTERRUPT;
}

/*! \brief  Universal receive interrupt service routine for both USART0 and the FTDI USB chip.
 *
 *  This routine is called whenever a new byte is available to be read. This service routine
 *	does also implement a sort of pre parsing of the incoming data stream. The stream is
 *  terminated when the EOT character is detected ('\n'). The serial interface will be
 *  stoped and the since the com_data_reception_finished flag set to true.
 *  The routine appends two characters at 
 */
/*ISR( USART0_RX_vect )
{
	
  	uint8_t receivedData;
	
	receivedData = ( uint8_t )UDR0;	//Collect data.
	
    if (com_number_of_received_bytes < COM_RX_MAX_BYTES) 
	{
        
        //End of data stream.
        if (receivedData == '\n') 
		{            
	        //DISABLE_RECEIVE_COMPLETE_INTERRUPT;
            com_buffer[com_number_of_received_bytes++] = receivedData;
            com_buffer[com_number_of_received_bytes++] = 0x00;
            com_buffer[com_number_of_received_bytes++] = 0x00;
            
            com_data_reception_finished = true;
        } 
		else 
		{
            com_buffer[com_number_of_received_bytes++] = receivedData;
        }
    }
    
    else
	{        
        //DISABLE_RECEIVE_COMPLETE_INTERRUPT;
        com_number_of_received_bytes = 1;
        com_data_reception_finished = true;
    }
}*/

ISR( USART0_RX_vect )
{

	uint8_t receivedData;

//	TCCR0B = 0x00;
//	TCNT0 = Timer0_Initvalue;//0x19; //reset timer0 Value
//	hal_clear_timer0_flag();
	receivedData = ( uint8_t )UDR0;	//Collect data.

	if (com_number_of_received_bytes < COM_RX_MAX_BYTES) 
	{
	//End of data stream.
			if( (receivedData == '\n') ||(receivedData == 0x0D) )
			{
				/*if(com_number_of_received_bytes>1)
				{
					Set_com_Rx_Finished_flag();
					//DISABLE_RECEIVE_COMPLETE_INTERRUPT0;
				}*/
				com_number_of_received_bytes++;
				*com_buffer++= receivedData;
				com_number_of_received_bytes++;
				*com_buffer++=0;
				com_number_of_received_bytes++;
				*com_buffer++=0;
				com_data_reception_finished = true;
				if(ori_buf_flag){
					ori_buf_flag=false;
					com_buffer=com_buffer0;
					com_number_of_received_bytesB = com_number_of_received_bytes;
					//return com_buffer1;
				}else{
					ori_buf_flag=true;
					com_buffer=com_buffer1;
					com_number_of_received_bytesA = com_number_of_received_bytes;
					//return com_buffer0;
				}
					com_number_of_received_bytes=0;
			}
			else
			{
				com_number_of_received_bytes++;
				*com_buffer++= receivedData;
			}
		
	}
	else
	{
	        com_number_of_received_bytes = 1;
	        com_data_reception_finished = true;
/*		*com_buffer++ = receivedData;
		//DISABLE_RECEIVE_COMPLETE_INTERRUPT0;
		if(ori_buf_flag){
			ori_buf_flag=false;
			com_buffer=com_buffer0;
		}else{
			ori_buf_flag=true;
			com_buffer=com_buffer1;
		}
		com_number_of_received_bytes = 0;
		Set_com_Rx_Finished_flag();
	}
	TCCR0B = 0x04;
	
*/
	}
}



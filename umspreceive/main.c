/*============================ INCLDUE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include <clock_config.h>
#include "config_uart_extended.h" /* See this file for all project options. */
#include "compiler.h"
#include "at86rf231.h"
#include "hal.h"
#include "tat.h"
#include "com.h"
#include "hal_avr.h"
/*============================ MACROS ========================================*/
/*============================ TYPEDEFS ======================================*/
/*============================ VARIABLES =====================================*/


static hal_rx_frame_t	rx_pool[RX_POOL_SIZE];             /* !< Pool of hal_rx_frame_t's. */
static hal_rx_frame_t	*rx_pool_start;                    /* !< Pointer to start of pool. */
static hal_rx_frame_t	*rx_pool_end;                      /* !< Pointer to end of pool. */
static hal_rx_frame_t	*rx_pool_head;                    /* !< Pointer to next hal_rx_frame_t it is possible to write. */
static hal_rx_frame_t	*rx_pool_tail;                    /* !< Pointer to next hal_rx_frame_t that can be read from the pool. */
static uint8_t		rx_pool_items_free;                     /* !< Number of free items (hal_rx_frame_t) in the pool. */
static uint8_t		rx_pool_items_used;                   /* !< Number of used items. */
static bool		rx_pool_overflow_flag;                      /* !< Flag that is used to signal a pool overflow. */

static bool rx_flag;                                      /* !< Flag used to mask between the two possible TRX_END events. */


static uint8_t	debug_type_message[]		= "\r<---Type Message:\r\n";                    /* !< Debug Text. */
static uint8_t	debug_rx_pool_overflow[]	= "RX Buffer Overflow!\r\n";                    /* !< Debug Text. */
static uint8_t	debug_transmission_failed[]	= "TX Failed!\r\n";                             /* !< Debug Text. */
static uint8_t	debug_fatal_error[]		= "A fatal error. System must be reset.\r\n";   /* !< Debug Text. */
/*============================ PROTOTYPES ====================================*/
static bool trx_init( void );


static void avr_init( void );


static void trx_end_handler( uint32_t time_stamp );


static void rx_pool_init( void );


/*! \brief This function is used to initialize the TRX.
 *
 * The TAT will be set up to run on the chosen operating channel, with CLKM diabled,
 * and then configure the RX_AACK and TX_ARET modes.
 *
 *  \retval true if the TRX was successfully configured.
 *  \retval false if the TRX was not configured properly.
 */
static bool trx_init( void )
{
	static bool status;

	if ( tat_init() != TAT_SUCCESS )
	{
		status = false;
	} else if ( tat_set_operating_channel( OPERATING_CHANNEL ) != TAT_SUCCESS )
	{
		status = false;
	} else if ( tat_set_clock_speed( true, CLKM_NO_CLOCK ) != TAT_SUCCESS )
	{
		status = false;
	} else{
		/*Set up the extended modes:*/
		/* RX_AACK: */
		hal_subregister_write( SR_OQPSK_DATA_RATE, ALTRATE_250KBPS );
		hal_subregister_write( SR_ANT_DIV_EN, ANT_DIV_DISABLE );
		hal_subregister_write( SR_ANT_EXT_SW_EN, ANT_EXT_SW_SWITCH_DISABLE );
		tat_set_short_address( SHORT_ADDRESS );                 /* Short Address. */
		tat_set_pan_id( PAN_ID );                               /* PAN ID. */
		tat_set_device_role( false );                           /* No Coordintor support is necessary. */

		/* TX_ARET: */
		tat_configure_csma( 234, 0xE2 );                        /* Default CSMA_SEED_0, MIN_BE = 3, MAX_CSMA_RETRIES = , and CSMA_SEED_1 = */

		/* Both Modes: */
		tat_use_auto_tx_crc( true );                            /* Automatic CRC must be enabled. */
		hal_set_trx_end_event_handler( trx_end_handler );       /* Event handler for TRX_END events. */

		status = true;
	} /* end: if (tat_init( ) != TAT_SUCCESS) ... */

	return(status);
}


/*! \brief This function configure the necessary IO modules on the AVR.
 */
static void avr_init( void )
{
	com_init( BR_9600 );
}


/*! \brief This function initialize the rx_pool. The rx_pool is in essence a FIFO.
 */
static void rx_pool_init( void )
{
	rx_pool_start	= rx_pool;
	rx_pool_end	= &rx_pool[RX_POOL_SIZE - 1];

	rx_pool_head	= rx_pool_start;
	rx_pool_tail	= rx_pool_end;

	rx_pool_items_free	= RX_POOL_SIZE;
	rx_pool_items_used	= 0;

	rx_pool_overflow_flag = false;
}


/*! \brief This function is the TRX_END event handler that is called from the
 *         TRX isr if assigned.
 *
 *  \param[in] time_stamp Interrupt timestamp in IEEE 802.15.4 symbols.
 */
static void trx_end_handler( uint32_t time_stamp )
{
	if ( rx_flag == true )
	{
		/* Check if these is space left in the rx_pool. */
		if ( rx_pool_items_free == 0 )
		{
			rx_pool_overflow_flag = true;
		} else {
			/* Space left, so upload the received frame. */
			hal_frame_read( rx_pool_head );

			/* Then check the CRC. Will not store frames with invalid CRC. */
			if ( rx_pool_head->crc == true )
			{
				/* Handle wrapping of rx_pool. */
				if ( rx_pool_head == rx_pool_end )
				{
					rx_pool_head = rx_pool_start;
				} else {
					++rx_pool_head;
				}       /* end: if (rx_pool_head == rx_pool_end) ... */

				--rx_pool_items_free;
				++rx_pool_items_used;
			}               /* end: if (rx_pool_head->crc == true) ... */
		}                       /* end: if (rx_pool_items_free == 0) ... */
	}                               /* end:  if (rx_flag == true) ... */
}


int main( void )
{
	static uint8_t	length_of_received_data = 0;
	static uint8_t	frame_sequence_number	= 0;
	rx_flag = true;
	rx_pool_init();
	avr_init();
	trx_init();

	/* Set system state to RX_AACK_ON */
	if ( tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS )
	{
		com_send_string( debug_fatal_error, sizeof(debug_fatal_error) );
	} /* end: if (tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS) ... */

	sei();
	hal_set_net_led();

	/* Give the user an indication that the system is ready. */
	com_send_string( debug_type_message, sizeof(debug_type_message) );
	length_of_received_data = hal_register_read( RG_PART_NUM );
	frame_sequence_number	= hal_register_read( RG_VERSION_NUM );


	/*Enter Normal Program Flow:
	 *   - Check for newly received frames. Print them if something is received.
	 *   - Notify on rx_pool overflow.
	 *   - Try to send data on air interface, if something is received on UART/USB.
	 *   - Notify if the typed message was too long.
	 */
	while ( true )
	{
		/* Check if we have received something on the air interface. */
		if ( rx_pool_items_used != 0 )
		{
			hal_set_data_led();
			/* Handle wrapping of rx_pool. */
			if ( rx_pool_tail == rx_pool_end )
			{
				rx_pool_tail = rx_pool_start;
			} else {
				++rx_pool_tail;
			} /* end: if (rx_pool_tail == rx_pool_end) ... */

			/*
			 * Turn interrupts off for a short while to protect when status
			 * information about the rx_pool is updated.
			 */
			cli();

			++rx_pool_items_free;
			--rx_pool_items_used;

			sei();

			/* Send the frame to the user: */
			static uint8_t space[] = "  ";
			/* com_send_string( debug_data_received, sizeof( debug_data_received ) ); */
			DDRF	|= 1 << 2;
			PORTF	&= ~(1 << 2);
			com_send_hex( rx_pool_tail->data[10] );
			com_send_hex( rx_pool_tail->data[9] );
			com_send_hex( rx_pool_tail->length );
			com_send_hex( rx_pool_tail->data[4] );
			com_send_hex( rx_pool_tail->data[3] );
			com_send_hex( rx_pool_tail->data[19] );
			com_send_hex( rx_pool_tail->data[2] );
			com_send_hex( rx_pool_tail->data[6] );
			com_send_hex( rx_pool_tail->data[5] );
			com_send_hex( rx_pool_tail->data[8] );
			com_send_hex( rx_pool_tail->data[7] );
			com_send_hex( rx_pool_tail->data[12] );
			com_send_hex( rx_pool_tail->data[13] );
			com_send_hex( rx_pool_tail->data[14] );
			com_send_hex( rx_pool_tail->data[16] );
			com_send_hex( rx_pool_tail->data[15] );
			com_send_hex( rx_pool_tail->data[18] );
			com_send_hex( rx_pool_tail->data[17] );
			hal_clear_data_led();
		} /* end: if (rx_pool_items_used != 0) ... */

		/* Check for rx_pool overflow. */
		if ( rx_pool_overflow_flag == true )
		{
			cli();
			rx_pool_init();
			com_send_string( debug_rx_pool_overflow, sizeof(debug_rx_pool_overflow) );
			sei();
		}       /* end: if (rx_pool_overflow_flag == true) ... */

		/*
		 * Check for new data on the serial interface.
		 * Check if data is ready to be sent.
		 * length_of_received_data = com_get_number_of_received_bytes( );
		 */
	}               /* emd: while (true) ... */
	return(0);
}


/*EOF*/

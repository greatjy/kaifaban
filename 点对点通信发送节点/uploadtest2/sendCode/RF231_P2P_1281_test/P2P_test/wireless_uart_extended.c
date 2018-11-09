/*============================ INCLDUE =======================================*/
#include <stdint.h>
#include <stdbool.h>

#include "config_uart_extended.h" // See this file for all project options.

#include "compiler.h"

#include "at86rf231.h"
#include "hal.h"
#include "tat.h"
#include "com.h"
#include "hal_avr.h"
/*============================ MACROS ========================================*/
/*============================ TYPEDEFS ======================================*/
/*============================ VARIABLES =====================================*/
static uint8_t tx_frame[ 127 ]; //!< Buffer used to build TX frames. (Size must be max PSDU length.)
static uint8_t tx_frame_info[16];//新建数组用来向串口发送数据
static hal_rx_frame_t rx_pool[ RX_POOL_SIZE ]; //!< Pool of hal_rx_frame_t's.
static hal_rx_frame_t *rx_pool_start; //!< Pointer to start of pool.
static hal_rx_frame_t *rx_pool_end; //!< Pointer to end of pool.
static hal_rx_frame_t *rx_pool_head; //!< Pointer to next hal_rx_frame_t it is possible to write.
static hal_rx_frame_t *rx_pool_tail; //!< Pointer to next hal_rx_frame_t that can be read from the pool.
static uint8_t rx_pool_items_free; //!< Number of free items (hal_rx_frame_t) in the pool.
static uint8_t rx_pool_items_used; // !< Number of used items.
static bool rx_pool_overflow_flag; //!< Flag that is used to signal a pool overflow.

static bool rx_flag; //!< Flag used to mask between the two possible TRX_END events.

static uint8_t debug_pll_transition[] = "State transition failed\r\n"; //!< Debug Text.
static uint8_t debug_type_message[] = "\r<---Type Message:\r\n"; //!< Debug Text.
static uint8_t debug_data_sent[] = "<---TX OK.\r\n"; //!< Debug Text.
static uint8_t debug_data_received[] = "\r--->Rx:\r"; //!< Debug Text.
static uint8_t debug_lqi[] = "LQI: "; //!< Debug Text.
static uint8_t debug_rx_pool_overflow[] = "RX Buffer Overflow!\r\n"; //!< Debug Text.
static uint8_t debug_transmission_failed[] = "TX Failed!\r\n"; //!< Debug Text.
static uint8_t debug_transmission_length[] = "Typed Message too long!!\r\n"; //!< Debug Text.
static uint8_t debug_fatal_error[] = "A fatal error. System must be reset.\r\n"; //!< Debug Text.
/*============================ PROTOTYPES ====================================*/
static bool trx_init( void );
static void avr_init( void );
static void trx_end_handler( uint32_t time_stamp );
static void rx_pool_init( void );

static uint8_t length_of_received_data = 20;
static uint8_t tx_frame_length = 22;

/*! \brief This function is used to initialize the TRX.
 *
 * The TAT will be set up to run on the chosen operating channel, with CLKM diabled,
 * and then configure the RX_AACK and TX_ARET modes.
 *
 *  \retval true if the TRX was successfully configured.
 *  \retval false if the TRX was not configured properly.
 */
static bool trx_init( void ){

    static bool status;

    if (tat_init( ) != TAT_SUCCESS) {
        status = false;
    } else if (tat_set_operating_channel( OPERATING_CHANNEL ) != TAT_SUCCESS) {
        status = false;
    } else if (tat_set_clock_speed( true, CLKM_NO_CLOCK ) != TAT_SUCCESS) {
        status = false;
    } else{

        /*Set up the extended modes:*/
        //RX_AACK:
        hal_subregister_write(SR_OQPSK_DATA_RATE, ALTRATE_250KBPS);
        hal_subregister_write(SR_ANT_DIV_EN, ANT_DIV_DISABLE );
     hal_subregister_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_SWITCH_DISABLE);
        tat_set_short_address( SHORT_ADDRESS ); //Short Address.
        tat_set_pan_id( PAN_ID ); //PAN ID.
        tat_set_device_role( false ); // No Coordintor support is necessary.

        //TX_ARET:
        tat_configure_csma( 234, 0xE2 ); // Default CSMA_SEED_0, MIN_BE = 3, MAX_CSMA_RETRIES = , and CSMA_SEED_1 =

        //Both Modes:
        tat_use_auto_tx_crc( true ); //Automatic CRC must be enabled.
        hal_set_trx_end_event_handler( trx_end_handler ); // Event handler for TRX_END events.

        status = true;
    } // end: if (tat_init( ) != TAT_SUCCESS) ...

    return status;
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

    rx_pool_start = rx_pool;
    rx_pool_end = &rx_pool[ RX_POOL_SIZE - 1 ];

    rx_pool_head = rx_pool_start;
    rx_pool_tail = rx_pool_end;

    rx_pool_items_free = RX_POOL_SIZE;
    rx_pool_items_used = 0;

    rx_pool_overflow_flag = false;
}

/*! \brief This function is the TRX_END event handler that is called from the
 *         TRX isr if assigned.
 *
 *  \param[in] time_stamp Interrupt timestamp in IEEE 802.15.4 symbols.
 */
static void trx_end_handler( uint32_t time_stamp ){

    if (rx_flag == true) {

        //Check if these is space left in the rx_pool.
        if (rx_pool_items_free == 0) {
            rx_pool_overflow_flag = true;
        } else {

            //Space left, so upload the received frame.
            hal_frame_read( rx_pool_head );

            //Then check the CRC. Will not store frames with invalid CRC.
            if (rx_pool_head->crc == true) {

                //Handle wrapping of rx_pool.
                if (rx_pool_head == rx_pool_end) {
                    rx_pool_head = rx_pool_start;
                } else {
                    ++rx_pool_head;
                } // end: if (rx_pool_head == rx_pool_end) ...

                --rx_pool_items_free;
                ++rx_pool_items_used;
            } // end: if (rx_pool_head->crc == true) ...
        } // end: if (rx_pool_items_free == 0) ...
    } // end:  if (rx_flag == true) ...
}
/*static void convert_16_bit_to_byte_array(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}*/
/*
构建IEEE 802.15.4 数据帧用于发送
*/
static void configureFrameSending()
{

/*Pre Build Header of IEEE 802.15.4 Data frame.*/
    tx_frame[ 0 ] = 0x61; //FCF.
    tx_frame[ 1 ] = 0x88; //FCF.
                           //Sequence number set during frame transmission.
    tx_frame[ 3 ] = PAN_ID & 0xFF; //Dest. PANID.
    tx_frame[ 4 ] = (PAN_ID >> 8 ) & 0xFF; //Dest. PANID.
    tx_frame[ 5 ] = DEST_ADDRESS & 0xFF; //Dest. Addr.
    tx_frame[ 6 ] = (DEST_ADDRESS >> 8 ) & 0xFF; //Dest. Addr.
    tx_frame[ 7 ] = SHORT_ADDRESS & 0xFF; //Source Addr.
    tx_frame[ 8 ] = (SHORT_ADDRESS >> 8 ) & 0xFF; //Source Addr.
    /*start symbol*/
	tx_frame[9] = 0x0DB5 & 0xFF;
	tx_frame[10] = (0x0DB5>>8)&0xFF;
	  /*data length*/
	//frame_ptr++ = 3//DEFAULT_FRAME_LENGTH - FRAME_OVERHEAD - 6  20-11-6
	tx_frame[11] = 3;
	  /*data field*/
    tx_frame[12] = 0x2B;
	tx_frame[13] = 0x31;
	tx_frame[14] = 0x31;
	tx_frame[15] = 0xFF;
	tx_frame[16] = 0xFF;
	  /*end symbol*/
   	tx_frame[17] =  0x0CD5&0xFF;
	tx_frame[18] =  (0x0CD5>>8)&0xFF;
}
/* \brief This function  passes the data to the serial port.
*   将发送信息打印在串口上
*
*/
static void uploadPrint()
{
     tx_frame_info[0] = (0x0DB5>>8)&0xFF;
     tx_frame_info[1] = 0x0DB5 & 0xFF;
     tx_frame_info[2] = 3;
	 tx_frame_info[3] = 0x01;
	 tx_frame_info[4] = 0x02;
	 tx_frame_info[5] = tx_frame[19];
	 tx_frame_info[6] = tx_frame[2];
	 tx_frame_info[7] = 0x02;
	 tx_frame_info[8] = 0x01;
	 tx_frame_info[9] = tx_frame[12];
	 tx_frame_info[10] = tx_frame[13];
	 tx_frame_info[11] = tx_frame[14];
	 tx_frame_info[12] = 0xFF;
	 tx_frame_info[13] = 0xFF;
	 tx_frame_info[14] = (0x0CD5>>8)&0xFF;
	 tx_frame_info[15] = (0x0CD5)&0xFF;
     static uint8_t space[]=" ";
     for(int i=0;i<16;i++)
	 {
	     com_send_hex(tx_frame_info[i]);
		 com_send_string(space,sizeof(space));
	 }

}
int main( void ){

    static uint8_t length_of_received_data = 0;
    static uint8_t frame_sequence_number = 0;
	static uint8_t frame_carry = 0;
    rx_flag = true;
    configureFrameSending();

    rx_pool_init( );
    avr_init( );
    trx_init( );

    //Set system state to RX_AACK_ON
    if (tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS) {
        com_send_string( debug_fatal_error, sizeof( debug_fatal_error ) );
    } // end: if (tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS) ...

    sei( );
    hal_set_net_led();
    //Give the user an indication that the system is ready.
    com_send_string( debug_type_message, sizeof( debug_type_message ) );
    length_of_received_data = hal_register_read(RG_PART_NUM);
    frame_sequence_number = hal_register_read(RG_VERSION_NUM );
   /*Enter Normal Program Flow:
        - Check for newly received frames. Print them if something is received.
        - Notify on rx_pool overflow.
        - Try to send data on air interface, if something is received on UART/USB.
        - Notify if the typed message was too long.
     */
    while (true)
	{
        if (rx_pool_overflow_flag == true) {

            cli();
            rx_pool_init( );
            com_send_string( debug_rx_pool_overflow, sizeof( debug_rx_pool_overflow ) );
            sei();
        }
        length_of_received_data = 20;
        if (length_of_received_data == 1) {
            com_send_string( debug_transmission_length, sizeof( debug_transmission_length ) );
            com_reset_receiver( );
        } else {

            if ((length_of_received_data >= 3) && (length_of_received_data <= COM_RX_BUFFER_SIZE)) {

                //Change state to TX_ARET_ON and send data if the state transition was successful.
                //发送函数
                if (tat_set_trx_state( TX_ARET_ON ) == TAT_SUCCESS) {
				    uploadPrint();
                    //uint8_t tx_frame_length = 19; // Length of prebuilt frame header.
                    //记录发送成功的帧的个数
                    frame_sequence_number++; //Sequence Number.
                    if(frame_sequence_number == 255)
                   	{
                         frame_sequence_number = 0;
						 frame_carry++;
					}
					tx_frame[2] = frame_sequence_number;
					tx_frame[19] = frame_carry;
                    hal_set_data_led();
                    //Copy data into the TX frame buffer.
                    rx_flag = false; // Set the flag false, so that the TRX_END event is not misinterpreted.

					//发送寄存器中的值到发送节点
                    if (tat_send_data_with_retry( tx_frame_length, tx_frame, 1 ) == TAT_SUCCESS) {
                        //com_send_string( debug_data_sent, sizeof( debug_data_sent ) );
                    } else {
                        com_send_string( debug_transmission_failed, sizeof( debug_transmission_failed ) );
                    } // end:  if (tat_send_data_with_retry( tx_frame_length, tx_frame, 1 ) ...
                } else {
                    com_send_string( debug_pll_transition, sizeof( debug_pll_transition ) );
                } // end: if (tat_set_trx_state( TX_ARET_ON ) == TAT_SUCCESS) ...

                if (tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS) {
                    com_send_string( debug_fatal_error, sizeof( debug_fatal_error ) );
                } // end: if (tat_set_trx_state( RX_AACK_ON ) != TAT_SUCCESS) ...

                rx_flag = true; // Set the flag back again. Only used to protec the frame transmission.
                com_reset_receiver( );
                //com_send_string( debug_type_message, sizeof( debug_type_message ) );
                hal_clear_data_led();
		        delay_us(100000);
            } // end:
        } // end: if (length_of_received_data == 1) ...*/
    } // emd: while (true) ...
    return 0;
}
/*EOF*/

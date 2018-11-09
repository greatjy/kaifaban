#ifndef TAT_H
#define TAT_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>

#include "compiler.h"
/*============================ MACROS ========================================*/
#define SUPPORTED_MANUFACTURER_ID               ( 31 )
#define RF231_SUPPORTED_INTERRUPT_MASK          ( 0xFF )

#define RF231_MIN_CHANNEL                       ( 11 )
#define RF231_MAX_CHANNEL                       ( 26 )
#define RF231_MIN_ED_THRESHOLD                  ( 0 )
#define RF231_MAX_ED_THRESHOLD                  ( 15 )
#define RF231_MAX_TX_FRAME_LENGTH               ( 127 ) //!< 127 Byte PSDU.

#define TX_PWR_3DBM                             ( 0 )
#define TX_PWR_17_2DBM                          ( 15 )

#define BATTERY_MONITOR_HIGHEST_VOLTAGE         ( 15 )
#define BATTERY_MONITOR_VOLTAGE_UNDER_THRESHOLD ( 0 )
#define BATTERY_MONITOR_HIGH_VOLTAGE            ( 1 )
#define BATTERY_MONITOR_LOW_VOLTAGE             ( 0 )

#define FTN_CALIBRATION_DONE                    ( 0 )
#define PLL_DCU_CALIBRATION_DONE                ( 0 )
#define PLL_CF_CALIBRATION_DONE                 ( 0 )
/*============================ TYPEDEFS ======================================*/

/*! \brief  This macro defines the start value for the TAT_* status constants.
 *          
 *          It was chosen to have this macro so that the user can define where
 *          the status returned from the TAT starts. This can be useful in a
 *          system where numerous drivers are used, and some range of status codes
 *          are occupied.
 * 
 *  \see tat_status_t
 *  \ingroup tat
 */
#define TAT_STATUS_START_VALUE                  ( 0x40 )

/*! \brief  This enumeration defines the possible return values for the TAT API 
 *          functions.
 *          
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 *  \ingroup tat
 */
typedef enum{
    //!< The requested service was performed successfully.
    TAT_SUCCESS = TAT_STATUS_START_VALUE,
    //!< The connected device is not an Atmel AT86RF230.
    TAT_UNSUPPORTED_DEVICE,
    //!< One or more of the supplied function arguments are invalid.
    TAT_INVALID_ARGUMENT,
    //!< The requested service timed out.
    TAT_TIMED_OUT,
    //!< The end-user tried to do an invalid state transition.
    TAT_WRONG_STATE,
    //!< The radio transceiver is busy receiving or transmitting.
    TAT_BUSY_STATE,
    //!< The requested state transition could not be completed.
    TAT_STATE_TRANSITION_FAILED,
    //!< Channel in idle. Ready to transmit a new frame.
    TAT_CCA_IDLE,
    //!< Channel busy.
    TAT_CCA_BUSY,
    //!< Transceiver is busy receiving or transmitting data.
    TAT_TRX_BUSY,
    //!< Measured battery voltage is lower than voltage threshold.
    TAT_BAT_LOW,
    //!< Measured battery voltage is above the voltage threshold.
    TAT_BAT_OK,
    //!< The CRC failed for the actual frame.
    TAT_CRC_FAILED,
    //!< The channel access failed during the auto mode.
    TAT_CHANNEL_ACCESS_FAILURE,
    //!< No acknowledge frame was received.
    TAT_NO_ACK,
}tat_status_t;

/*! \brief  This enumeration defines the possible modes available for the 
 *          Clear Channel Assessment algorithm.
 *
 *          These constants are extracted from the datasheet.
 *
 *  \ingroup tat
 */
typedef enum{
    //!< Use energy detection above threshold mode.
    CCA_ED                    = 0,
    //!< Use carrier sense mode.
    CCA_CARRIER_SENSE         = 1,
    //!< Use a combination of both energy detection and carrier sense.
    CCA_CARRIER_SENSE_WITH_ED = 2
}tat_cca_mode_t;

/*============================ PROTOTYPES ====================================*/
tat_status_t tat_init( void );
uint8_t tat_get_operating_channel( void );
tat_status_t tat_set_operating_channel( uint8_t channel );
uint8_t tat_get_tx_power_level( void );
tat_status_t tat_set_tx_power_level( uint8_t power_level );

tat_status_t tat_do_ed_scan( uint8_t *ed_level );
uint8_t tat_get_cca_mode( void );
uint8_t tat_get_ed_threshold( void );
tat_status_t tat_set_cca_mode( uint8_t mode, uint8_t ed_threshold );
tat_status_t tat_do_cca( void );
tat_status_t  tat_get_rssi_value( uint8_t *rssi );

uint8_t tat_batmon_get_voltage_threshold( void );
uint8_t tat_batmon_get_voltage_range( void );
tat_status_t tat_batmon_configure( bool range, uint8_t voltage_threshold );
tat_status_t tat_batmon_get_status( void );

uint8_t tat_get_clock_speed( void );
tat_status_t tat_set_clock_speed( bool direct, uint8_t clock_speed );
tat_status_t tat_calibrate_filter( void );
tat_status_t tat_calibrate_pll( void );

uint8_t tat_get_trx_state( void );
tat_status_t tat_set_trx_state( uint8_t new_state );
tat_status_t tat_enter_sleep_mode( void );
tat_status_t tat_leave_sleep_mode( void );
void tat_reset_state_machine( void );
void tat_reset_trx( void );

void tat_use_auto_tx_crc( bool auto_crc_on );
__x tat_status_t tat_send_data( uint8_t data_length, uint8_t *data );

uint8_t tat_get_device_role( void );
void tat_set_device_role( bool i_am_coordinator );
uint16_t tat_get_pan_id( void );
void tat_set_pan_id( uint16_t new_pan_id );
uint16_t tat_get_short_address( void );
void tat_set_short_address( uint16_t new_short_address );
__x void tat_get_extended_address( uint8_t *extended_address );
__x void tat_set_extended_address( uint8_t *extended_address );
tat_status_t tat_configure_csma( uint8_t seed0, uint8_t be_csma_seed1 );
__x tat_status_t tat_send_data_with_retry( uint8_t frame_length, uint8_t *frame, 
                                       uint8_t retries );
#endif
/*EOF*/

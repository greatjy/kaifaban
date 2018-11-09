#ifndef HAL_AVR_H
#define HAL_AVR_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
/*============================ MACROS ========================================*/

/*! \brief Pin number that corresponds to the SLP_TR pin.
 * 
 *  \ingroup hal_avr_board
 */
#define SLP_TR              ( 0x04 )

/*! \brief Data Direction Register that corresponds to the port where SLP_TR is 
 *         connected.
 *  \ingroup hal_avr_board
 */
#define DDR_SLP_TR          ( DDRB )

/*! \brief Port (Write Access) where SLP_TR is connected.
 *  \ingroup hal_avr_board
 */
#define PORT_SLP_TR         ( PORTB )

/*! \brief Pin (Read Access) where SLP_TR is connected.
 *  \ingroup hal_avr_board
 */
#define PIN_SLP_TR          ( PINB )        

/*! \brief This macro pulls the SLP_TR pin high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_set_slptr_high( )         ( PORT_SLP_TR |= ( 1 << SLP_TR ) )

/*! \brief This macro pulls the SLP_TR pin low.
 *
 *  \ingroup hal_avr_api
 */
#define hal_set_slptr_low( )          ( PORT_SLP_TR &= ~( 1 << SLP_TR ) )

/*! \brief  Read current state of the SLP_TR pin (High/Low).
 *
 *  \retval 0 if the pin is low, 1 is the pin is high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_get_slptr( ) ( ( PIN_SLP_TR & ( 1 << SLP_TR ) ) >> SLP_TR )

/*! \brief Pin number that corresponds to the RST pin.
 *  \ingroup hal_avr_board
 */
#define RST                 ( 0x05 )

/*! \brief Data Direction Register that corresponds to the port where RST is 
 *         connected.
 *  \ingroup hal_avr_board
 */
#define DDR_RST             ( DDRB )

/*! \brief Port (Write Access) where RST is connected.
 *  \ingroup hal_avr_board
 */
#define PORT_RST            ( PORTB )

/*! \brief Pin (Read Access) where RST is connected.
 *  \ingroup hal_avr_board
 */
#define PIN_RST             ( PINB )

/*! \brief This macro pulls the RST pin high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_set_rst_high( )           ( PORT_RST |= ( 1 << RST ) )

/*! \brief This macro pulls the RST pin low.
 *
 *  \ingroup hal_avr_api
 */
#define hal_set_rst_low( )            ( PORT_RST &= ~( 1 << RST ) )

/*! \brief  Read current state of the RST pin (High/Low).
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_get_rst( ) ( ( PIN_RST & ( 1 << RST )  ) >> RST )

/*! \brief The slave select pin is PB0.
 *  \ingroup hal_avr_board
 */
#define HAL_SS_PIN              ( 0x00 )

/*! \brief The SPI module is located on PORTB.
 *  \ingroup hal_avr_board
 */
#define HAL_PORT_SPI            ( PORTB )

/*! \brief Data Direction Register for PORTB.
 *  \ingroup hal_avr_board
 */
#define HAL_DDR_SPI             ( DDRB )

/*! \brief Data Direction bit for SS.
 *  \ingroup hal_avr_board
 */
#define HAL_DD_SS               ( 0x00 )

/*! \brief Data Direction bit for SCK.
 *  \ingroup hal_avr_board
 */
#define HAL_DD_SCK              ( 0x01 )

/*! \brief Data Direction bit for MOSI.
 *  \ingroup hal_avr_board
 */
#define HAL_DD_MOSI             ( 0x02 )

/*! \brief Data Direction bit for MISO.
 *  \ingroup hal_avr_board
 */
#define HAL_DD_MISO             ( 0x03 )

#define HAL_SS_HIGH( ) (HAL_PORT_SPI |= ( 1 << HAL_SS_PIN )) //!< MACRO for pulling SS high.
#define HAL_SS_LOW( )  (HAL_PORT_SPI &= ~( 1 << HAL_SS_PIN )) //!< MACRO for pulling SS low.

#define NET_LED           ( 0x01 )
#define HAL_PORT_NET_LED   ( PORTF )
#define HAL_DDR_NET_LED   (DDRF)

#define hal_clear_net_led()  ( HAL_PORT_NET_LED |= (1<< NET_LED))
#define hal_set_net_led() ( HAL_PORT_NET_LED &=~ (1 << NET_LED))


#define DATA_LED        ( 0x00 )
#define HAL_PORT_DATA_LED   ( PORTF )
#define HAL_DDR_DATA_LED   (DDRF)

#define hal_clear_data_led()  ( HAL_PORT_DATA_LED |= (1<< DATA_LED))
#define hal_set_data_led() ( HAL_PORT_DATA_LED &=~ (1 << DATA_LED))



/*! \brief Macros defined for HAL_TIMER1.
 *
 *  These macros are used to define the correct setupt of the AVR's Timer1, and
 *  to ensure that the hal_get_system_time function returns the system time in 
 *  symbols (16 us ticks).
 *
 *  \ingroup hal_avr_board
 */

#if ( F_CPU == 16000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS12 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 8000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#elif ( F_CPU == 7372800UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#elif ( F_CPU == 4000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 1000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#else
    #error "Clock speed not supported."
#endif

#define HAL_ENABLE_INPUT_CAPTURE_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) )
#define HAL_DISABLE_INPUT_CAPTURE_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ) )

#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TIMSK1 |= ( 1 << TOIE1 ) )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TIMSK1 &= ~( 1 << TOIE1 ) )

/*! \brief  Enable the interrupt from the radio transceiver.
 *
 *  \ingroup hal_avr_api
 */
#define hal_enable_trx_interrupt( ) ( HAL_ENABLE_INPUT_CAPTURE_INTERRUPT( ) )

/*! \brief  Disable the interrupt from the radio transceiver.
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_disable_trx_interrupt( ) ( HAL_DISABLE_INPUT_CAPTURE_INTERRUPT( ) )
/*============================ TYPDEFS =======================================*/
/*============================ PROTOTYPES ====================================*/

#endif
/*EOF*/

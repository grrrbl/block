#ifndef BLOCK_H
#define BLOCK_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

/* Manipulate bits */
#define set_bit(var, bit) ((var) |= (1 << (bit)))
#define clear_bit(var, bit) ((var) &= (unsigned)~(1 << (bit)))
#define toggle_bit(var,bit) ((var) ^= (1 << (bit)))

// Define Debounce Options
#define REPEAT_MASK				0xFF
//#define REPEAT_MASK				0xF0
#define REPEAT_DELAY			40

// Other Pins
#define STATUS_LED_PIN			PA0
#define BlOCK_DATA_LINE_OUT		PA1
#define BLOCK_DATA_LINE_IN		PA2
#define PIN_RUECKMELDER1 		PB2

// USI status flags 
#define USI_TRANSFER_COMPLETE	0
#define USI_DEBOUNCE_KEY   		1
#define USI_WRITE_COLLISION		2
#define USI_RCK         		3
#define USI_ACTIVE      		7

// USI port and pin definitions.
#define USI_OUT_REG	    PORTA	//!< USI port output register.
#define USI_IN_REG	    PINA	//!< USI port input register.
#define USI_DIR_REG	    DDRA	//!< USI port direction register.
#define USI_CLOCK_PIN	PA4	    //!< USI clock I/O pin.
#define USI_DATAIN_PIN	PA6  	//!< USI data input pin.
#define USI_DATAOUT_PIN	PA5 	//!< USI data output pin.
#define USI_RCK_PIN     PA3
#define USI_PL_PIN      PA7

/*  Speed configuration:
 *  Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2.
 *  Maximum = CPUSPEED / 64.
 *
 *  or time for one byte on spi sent
 *  time = (PRESCALER * (COMPAREVALUE+1) * 8) / CPUSPEED
 *     
 */
#define TC0_COMPARE_VALUE   31	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.

/*  Prescaler value converted to bit settings.
PRESCALER_VALUE    1  (1<<CS00)
PRESCALER_VALUE    8  (1<<CS01)
PRESCALER_VALUE   64  (1<<CS01)|(1<<CS00)
PRESCALER_VALUE  256  (1<<CS02)
PRESCALER_VALUE 1024  (1<<CS02)|(1<<CS00)
*/
#define TC0_PRESCALER (1<<CS02)

// BLOCK STATUS BITS
#define BLOCK_ZUGANKUNFT            0x01 //0
#define BLOCK_ERLAUBNIS_EMPFANGEN   0x02 //1
#define BLOCK_ERLAUBNIS_GEGEBEN     0x04 //2
#define BLOCK_EMPFANGEN             0x08 //3
#define BLOCK_GEGEBEN               0x10 //4
#define BLOCK_BELEGT                0x1E

// SHIFT REGISTER OUTPUT (LEDS)
#define LED_ERSATZSIGNAL        0x20
#define LED_AUSFAHRSIGNAL       0x80 //4
#define LED_EINFAHRSIGNAL       0x10 //7
#define LED_ZUGANKUNFT          0x40 //6
#define LED_ERLAUBNIS_EMPFANGEN 0x08
#define LED_ERLAUBNIS_GEGEBEN   0x04
#define LED_BLOCK_EMPFANGEN     0x01
#define LED_BLOCK_GEGEBEN       0x02

// SHIFT REGISTER INPUT (KEY)
#define KEY_ERSATZSIGNAL        0x02
#define KEY_HILFSTASTE_BLOCK    0x01
#define KEY_AUSFAHRSIGNAL       0x40
#define KEY_EINFAHRSIGNAL       0x80
#define KEY_ZUGANKUNFT          0x20
#define KEY_ERLAUBNIS           0x10

void spi_put( uint8_t val );
void spi_sr_set();
void spi_wait();
void spi_init();

void debounce_keys();
uint8_t get_key_press( uint8_t key_mask );
uint8_t get_key_rpt( uint8_t key_mask );
uint8_t get_key_short( uint8_t key_mask );
uint8_t get_key_long( uint8_t key_mask );

#endif

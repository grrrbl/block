/* Manipulate bits */
#define set_bit(var, bit) ((var) |= (1 << (bit)))
#define clear_bit(var, bit) ((var) &= (unsigned)~(1 << (bit)))
#define toggle_bit(var,bit) ((var) ^= (1 << (bit)))

// Define Debounce Options
#define REPEAT_MASK				0xFF
#define REPEAT_DELAY			50
#define KEY_PIN					PINB

// Other Pins
#define STATUS_LED_PIN			PA0
#define BlOCK_DATA_LINE_OUT		PA0
#define BLOCK_DATA_LINE_IN		PA0

// USI status flags 
#define USI_TRANSFER_COMPLETE	0
#define USI_WRITE_COLLISION		1

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
 */
#define TC0_COMPARE_VALUE   7	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.

/*  Prescaler value converted to bit settings.
PRESCALER_VALUE    1  (1<<CS00)
PRESCALER_VALUE    8  (1<<CS01)
PRESCALER_VALUE   64  (1<<CS01)|(1<<CS00)
PRESCALER_VALUE  256  (1<<CS02)
PRESCALER_VALUE 1024  (1<<CS02)|(1<<CS00)
*/
#define TC0_PRESCALER (1<<CS01)

// SHIFT REGISTER OUTPUT (LEDS)
#define LED_ERSATZSIGNAL        0x20
#define LED_AUSFAHRSIGNAL       0x10
#define LED_EINFAHRSIGNAL       0x80
#define LED_ZUGANKUNFT          0x40
#define LED_ERLAUBNIS_EMPFANGEN 0x04
#define LED_ERLAUBNIS_GEGEBEN   0x08
#define LED_BLOCK_EMPFANGEN     0x01
#define LED_BLOCK_GEGEBEN       0x02

// SHIFT REGISTER INPUT (KEY)
#define KEY_ERSATZSIGNAL        0x01
#define KEY_HILFSTASTE_BLOCK    0x02
#define KEY_AUSFAHRSIGNAL       0x80
#define KEY_EINFAHRSIGNAL       0x40
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


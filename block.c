#define     F_CPU   8000000UL

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "block.h"
//#include "bitio.h"

/* SPI */
volatile uint8_t usi_data;
volatile uint8_t usi_status;

ISR(TIM0_COMPA_vect){
    USICR |= (1<<USITC);	// Toggle clock output pin.
}

ISR(USI_OVF_vect){
	// disable compare match interrupt to prevent more USI counter clocks.
    TIMSK0 &= ~(1<<OCIE0A);
    
	// clear usi overflow interupt flag and clear USI counter
	USISR = (1<<USIOIF);
    usi_status = (1<<USI_TRANSFER_COMPLETE);

	// Copy USIDR to buffer to prevent overwrite on next transfer.
	usi_data = USIDR;
    
    // enable read shift register
    USI_OUT_REG &= ~(1<<USI_PL_PIN);
}

void spi_sr_set(){
    // Set data out on shift register
    USI_OUT_REG &= ~(1<<USI_RCK_PIN);
    USI_OUT_REG |= (1<<USI_RCK_PIN);
}

void spi_put( uint8_t val )
{
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
		if( (USISR & 0x0F) != 0 ) {
			// Indicate write collision and return.
			usi_status = (1<<USI_WRITE_COLLISION);
			return;
		}
	}

    usi_status ^= usi_status;
    
	// Put data in USI data register.
	USIDR = val;

    // disable read shift register
    USI_OUT_REG |= (1<<USI_PL_PIN);
    
	// Master should now enable compare match interrupts.
	TIFR0 |= (1<<OCF0A);   // Clear compare match flag.
	TIMSK0 |= (1<<OCIE0A); // Enable compare match interrupt.
}

void spi_wait()
{
	do {} while( usi_status & (~(1<<USI_TRANSFER_COMPLETE)));
}

void spi_init(){
	USI_DIR_REG |= (1<<USI_DATAOUT_PIN) | (1<<USI_CLOCK_PIN) | 
                   (1<<USI_RCK_PIN) | (1<<USI_PL_PIN);        // Outputs.
	USI_DIR_REG &= ~(1<<USI_DATAIN_PIN);                      // Inputs.
	USI_OUT_REG |= (1<<USI_DATAIN_PIN)|(1<<USI_PL_PIN)|(1<<USI_RCK_PIN)|(1<<USI_CLOCK_PIN);     // Pull-ups and Status.
	
	// Configure USI to 3-wire master mode with overflow interrupt.
	USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (1<<USICS0) | (1<<USICLK);

	// Enable 'Clear Timer on Compare match' and init prescaler.
	TCCR0A = (1<<WGM01); 
    TCCR0B = TC0_PRESCALER;

	// Init Output Compare Register.
	OCR0A = TC0_COMPARE_VALUE;
    
    usi_status = 0;
    usi_data   = 0;
}


/* DEBOUNCE */
volatile uint8_t check_debounce;
uint8_t key_state;				// debounced and inverted key state: bit = 1: key pressed
uint8_t key_press;				// key press detect
uint8_t key_press_long;         // key long press and repeat

//void debounce_keys(){
//  static uint8_t ct0, ct1, press_long;
//  uint8_t i;
//    
//  //TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);	// preload for 10ms
//
//  i = key_state ^ ~KEY_PIN;             // key changed ?
//  ct0 = ~( ct0 & i );                   // reset or count ct0
//  ct1 = ct0 ^ (ct1 & i);                // reset or count ct1
//  i &= ct0 & ct1;                       // count until roll over ?
//  key_state ^= i;                       // then toggle debounced state
//  key_press |= key_state & i;           // 0->1: key press detect
//
//  if( (key_state & REPEAT_MASK) == 0 )  // check repeat function
//     press_long = REPEAT_DELAY;
//  if( --press_long == 0 )
//    key_press_long |= key_state & REPEAT_MASK;
//}
//
//uint8_t get_key_press( uint8_t key_mask ){
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ 
//        key_mask &= key_press;         
//        key_press ^= key_mask;         
//    }
//  return key_mask;
//}
//
//uint8_t get_key_rpt( uint8_t key_mask ){
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ 
//        key_mask &= key_press_long;    
//        key_press_long ^= key_mask;    
//    }
//  return key_mask;
//}
//
//uint8_t get_key_short( uint8_t key_mask ){
//  return get_key_press( ~key_state & key_mask );
//}
//
//uint8_t get_key_long( uint8_t key_mask ){
//  return get_key_press( get_key_rpt( key_mask ));
//}

void main (void) {
    sei();                              // enable interupts

    /* Init SPI */
    spi_init();
    spi_put(0xF0);
    spi_wait();
    spi_sr_set();
    _delay_ms(1000);
    spi_put(0x0F);
    spi_wait();
    spi_sr_set();
    _delay_ms(1000);

    /* Init other stuff */
    DDRA    |=  (1 << PA0) ;              // Ausgang
    //DDRA    &= ~(1 << PB2);               // Eingang
    //PORTB   |=  (1 << PB2);               // Pullup
    PORTA   |= (1<<PA0);                  // lampe an

    while(1){
        static uint8_t put = 1;
        static uint8_t count = 0;
        
        if(usi_data & 0xF0){
            put = 0x0F;
            count = 255;
        }
        else if(usi_data & 0x0F){
            put = 0xF0;
            count = 255;
        }
        else if(count > 254)
        {
            put = put>>1;
            if(put == 0)
                put = 0x80;
        }
        spi_wait();
        spi_put(put);
        if(count > 254)
        {
            spi_wait();
            spi_sr_set();
            count=0;
        }
        count++;
        _delay_ms(1);
    }
}


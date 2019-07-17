#include "block.h"
#include "uart.h"

/* SPI */
volatile uint8_t usi_data, usi_status, slow_timer, fast_timer;

ISR(TIM0_COMPA_vect){
    if(usi_status & (1<<USI_ACTIVE))
        USICR |= (1<<USITC);	// Toggle clock output pin.
    if(++fast_timer > 4){
        slow_timer++;
        fast_timer ^= fast_timer;
    }
}

ISR(TIM1_COMPA_vect){
    tim1_compa_func();
}

ISR(TIM1_COMPB_vect){
    tim1_compb_func();
}

ISR(PCINT0_vect){
    pcint0_func();
}

ISR(USI_OVF_vect){
    //uint8_t usi_out_reg;
	// disable compare match interrupt to prevent more USI counter clocks.
    if (usi_status & (1<<USI_RCK))
        USI_OUT_REG |= (1<<USI_RCK_PIN);
        
    usi_status = (1<<USI_TRANSFER_COMPLETE);
    //usi_status &= ~(1<<USI_ACTIVE);
    
	// clear usi overflow interupt flag and clear USI counter
	USISR = (1<<USIOIF);

	// Copy USIDR to buffer to prevent overwrite on next transfer.
	usi_data = USIDR;
    
    // enable read shift register
    USI_OUT_REG &= ~(1<<USI_PL_PIN);
}

void spi_put( uint8_t val )
{
    static uint8_t val_ref = 0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
		if( (USISR & 0x0F) != 0 ) {
			// Indicate write collision and return.
			usi_status = (1<<USI_WRITE_COLLISION);
			return;
		}
	}

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        usi_status &= ~((1<<USI_WRITE_COLLISION)|(1<<USI_TRANSFER_COMPLETE));
    }
    
	// Put data in USI data register.
	USIDR = val;

    // disable read shift register
    USI_OUT_REG |=  (1<<USI_PL_PIN);
    USI_OUT_REG &= ~(1<<USI_RCK_PIN);
    
	// Master should now enable compare match interrupts.
	//TIFR0 |= (1<<OCF0A);   // Clear compare match flag.
	//TIMSK0 |= (1<<OCIE0A); // Enable compare match interrupt.
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        if (val_ref != val){
            usi_status |= (1<<USI_RCK);
            val_ref = val;
        }
        usi_status |= (1<<USI_ACTIVE);
    }
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

void debounce_keys(){
  static uint8_t ct0, ct1, press_long;
  uint8_t i;
    
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    //i = key_state ^ ~usi_data;             // key changed ?
    i = key_state ^ usi_data;             // key changed ?
  }
  ct0 = ~( ct0 & i );                   // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);                // reset or count ct1
  i &= ct0 & ct1;                       // count until roll over ?
  key_state ^= i;                       // then toggle debounced state
  key_press |= key_state & i;           // 0->1: key press detect

  if( (key_state & REPEAT_MASK) == 0 )  // check repeat function
     press_long = REPEAT_DELAY;
  if( --press_long == 0 )
    key_press_long |= key_state & REPEAT_MASK;
}

uint8_t get_key_press( uint8_t key_mask ){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ 
        key_mask &= key_press;         // check if button pressed
        key_press ^= key_mask;         // clear key_press
    }
  return key_mask;
}

uint8_t get_key_rpt( uint8_t key_mask ){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ 
        key_mask &= key_press_long;    
        key_press_long ^= key_mask;    
    }
  return key_mask;
}

uint8_t get_key_short( uint8_t key_mask ){
  
  return get_key_press( ~key_state & key_mask );
}

uint8_t get_key_long( uint8_t key_mask ){
    return get_key_press( get_key_rpt( key_mask ));
}

int main (void) {
    uart_init();
    //uart_putc("c");
    // Init PINs
    // Ausgänge
    DDRA |=  (1 << STATUS_LED_PIN)|(1 << BlOCK_DATA_LINE_OUT);
    // Eingänge
    DDRA &= ~(1 << BLOCK_DATA_LINE_IN);
    DDRB &= ~(1 << PIN_RUECKMELDER1);
    // Pullup / Status
    PORTB |=  (1 << PIN_RUECKMELDER1);               
    //PORTA   |= (1<<PA0)|(1 << BlOCK_DATA_LINE_OUT);

    // enable interupts
    sei();
    // Enable compare match interrupt.
	TIMSK0 |= (1<<OCIE0A);

    /* Init SPI and self test */
    spi_init();

    spi_put(0xF0);
    _delay_ms(500);
    spi_put(0x0F);
    _delay_ms(500);
    spi_put(0x00);

    while(1){
        static uint8_t block_status = 0;
        static uint8_t led_status = 0;

        if(get_key_short(KEY_AUSFAHRSIGNAL) && block_status & BLOCK_ERLAUBNIS_EMPFANGEN){
            uart_putc(0x55);
            led_status |= LED_AUSFAHRSIGNAL|LED_BLOCK_GEGEBEN;
            led_status &= ~LED_ERLAUBNIS_EMPFANGEN;
            block_status |= BLOCK_GEGEBEN;
            block_status &= ~BLOCK_ERLAUBNIS_EMPFANGEN;
        }
        else if(get_key_long(KEY_AUSFAHRSIGNAL))
            led_status &= ~LED_AUSFAHRSIGNAL;

        if(get_key_short(KEY_EINFAHRSIGNAL))
            led_status |= LED_EINFAHRSIGNAL;
        else if(get_key_long(KEY_EINFAHRSIGNAL))
            led_status &= ~LED_EINFAHRSIGNAL;

        if(get_key_long(KEY_HILFSTASTE_BLOCK)){
            uart_putc(0x11);
            led_status |= LED_ERLAUBNIS_EMPFANGEN;
            block_status |= BLOCK_ERLAUBNIS_EMPFANGEN;
        }

        if(get_key_short(KEY_ZUGANKUNFT) && block_status & BLOCK_ZUGANKUNFT){
            led_status &= ~(LED_ZUGANKUNFT | LED_BLOCK_EMPFANGEN);
            block_status &= ~(BLOCK_ZUGANKUNFT | BLOCK_EMPFANGEN);
        }

        if(get_key_short(KEY_HILFSTASTE_BLOCK) && block_status & BLOCK_EMPFANGEN){
            led_status |= LED_ZUGANKUNFT;
            block_status |= BLOCK_ZUGANKUNFT;
        }

        if(get_key_short(KEY_ERLAUBNIS) && !(block_status & BLOCK_BELEGT)){
            led_status |= LED_ERLAUBNIS_GEGEBEN;
            block_status |= BLOCK_ERLAUBNIS_GEGEBEN;
        }

        if(get_key_long(KEY_ERLAUBNIS)){
            led_status &= ~LED_ERLAUBNIS_GEGEBEN;
            block_status &= ~BLOCK_ERLAUBNIS_GEGEBEN;
        }
        
        if(usi_data & KEY_ERSATZSIGNAL){
            if(slow_timer <128 )
                led_status |= LED_ERSATZSIGNAL;
            else
                led_status &= ~LED_ERSATZSIGNAL;
        }
        else
            led_status &= ~LED_ERSATZSIGNAL;

        if(block_status & BLOCK_EMPFANGEN){
            led_status |= LED_BLOCK_EMPFANGEN;
            led_status &= ~LED_ERLAUBNIS_GEGEBEN;
        }

        if(!(PINB & (1<<PB2))){
            if(block_status & BLOCK_EMPFANGEN){
                led_status |= LED_ZUGANKUNFT;
                block_status |= BLOCK_ZUGANKUNFT;
            }
            else
                led_status &= ~LED_AUSFAHRSIGNAL;
        }

        // debounce keys, send data
        spi_wait();
        debounce_keys();
        spi_put(led_status);
        
        //testing
        if(block_status & BLOCK_ERLAUBNIS_GEGEBEN){
            block_status &= ~BLOCK_ERLAUBNIS_GEGEBEN;
            block_status |= BLOCK_EMPFANGEN;
            _delay_ms(1000);
        }

        //testing
        if(block_status & BLOCK_GEGEBEN){
            _delay_ms(2000);
            block_status &= ~BLOCK_GEGEBEN;
            led_status   &= ~LED_BLOCK_GEGEBEN;
        }
            

    }
    //_delay_ms(1000);

}


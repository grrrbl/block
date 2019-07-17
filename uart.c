#include "uart.h"

#ifdef USE_FIFO
#include "fifo.h"
#endif

volatile uint16_t tx_data, tx_data_ref;
volatile uint8_t bit_count, rx_data;
volatile uint8_t uart_status;

#ifdef USE_FIFO
    static uint8_t inbuf[INBUF_SIZE];
    fifo_t infifo;
    void fifo_init (fifo_t *f, uint8_t *buffer, const uint8_t size)
    {
        f->count = 0;
        f->read2end = f->write2end = f->size = size;
    }

        f->pread = f->pwrite = buffer;
    uint8_t fifo_put (fifo_t *f, const uint8_t data)
    {
        return _inline_fifo_put (f, data);
    }

    uint8_t fifo_get_wait (fifo_t *f)
    {
        while (!f->count);
        
        return _inline_fifo_get (f);    
    }

    int fifo_get_nowait (fifo_t *f)
    {
        if (!f->count)      return -1;
            
        return (int) _inline_fifo_get (f);  
    }
#else
    volatile uint8_t indata;
#endif

void uart_init(){
    // Mode für Timer1 
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12)| (1 << CS10);

    // OutputCompare für gewünschte Timer1 Frequenz 
    OCR1A = BAUDRATE;

    // RX Pin als Eingang
    SUART_RXD_DDR  &= ~(1 << SUART_RXD_BIT);
    // Interrupts on RX Pin
    GIMSK |= (1<<PCIE0);
    PCMSK0 |= (1<<SUART_RXD_BIT);

    // TX Pin als Ausgang
    SUART_TXD_DDR  |= (1 << SUART_TXD_BIT);

    tx_data = 0;
    tx_data_ref = 0;
    rx_data = 0;
    bit_count = 0;

#ifdef USE_FIFO
    // if fifo is used
    fifo_init (&infifo,   inbuf, INBUF_SIZE);
#endif
}

void uart_putc (uint8_t c)
{
    // wait for possible transmission to finish, 
    do {} while(uart_status & (1<<UART_STATUS_ACTIVE));
   
    // frame = *.P.7.6.5.4.3.2.1.0.S   S=Start(1), P=Stop(0), *=Endemarke(1) 
    tx_data = (1 << 10) | (c << 1) | (1 << 1);
    tx_data_ref = tx_data ;

    cli();
    // disable RX interrupts
    GIMSK &= ~(1<<PCIE0);
    // Set value and enable compare match interupt A   
    TIMSK1 |= (1 << OCIE1A);
    rx_data = 0;
    uart_status = 0;
    sei();
}


// send each bit
void tim1_compa_func(){
    // for collision management the sent bit should be read in and compared with read byte.
    // if there is a mismatch -> collision
    uint16_t tx_data_tmp = tx_data;
    //uint16_t rx_data_tmp = rx_data;

    // disable interupt flag on end of transmission
    if (tx_data_tmp & 1L){
        SUART_TXD_PORT |=  (1 << SUART_TXD_BIT);
        SUART_TXD_PORT |=  (1 << PA0);
    }
    else{
        SUART_TXD_PORT &= ~(1 << SUART_TXD_BIT);
        SUART_TXD_PORT &= ~(1 << PA0);
    }
    tx_data_tmp >>= 1;

    // read RX for checking write collision
    //if (SUART_RXD_PIN & (1 << SUART_RXD_BIT))
    //    rx_data_tmp |= (1 << 10);
    //tx_data_tmp >>= 1;

    if (tx_data_tmp == 1L){
        //if(tx_data_tmp != rx_data)
        //    uart_status |= UART_STATUS_COLLISION;
        uart_status |= UART_STATUS_TX_DONE;
        uart_status &= ~UART_STATUS_ACTIVE;
        TIMSK1 &= ~(1 << OCIE1A);
        SUART_TXD_PORT &= ~((1 << PA0)|(1 << SUART_TXD_BIT));
        GIMSK |= (1<<PCIE0);
        return;
    }

    tx_data = tx_data_tmp;
    //rx_data = rx_data_tmp;
}


/* receive bits
 * abtasten immer auf zeitintervall halbe
 */
void pcint0_func()
{
   // Pin change interupt beenden
    GIMSK &= ~(1<<PCIE0);
    GIFR = (1<<PCIF0);

    uint16_t tcnt1 = TCNT1;
    uint16_t ocr1a = OCR1A;
   

    // Compare match a value and activate interrupt 
    // Eine halbe Bitzeit zu ICR1 addieren (modulo OCR1A) und nach OCR1B
    uint16_t ocr1b;
    ocr1b = tcnt1 + ocr1a/2;
    if (ocr1b >= ocr1a)
        ocr1b -= ocr1a;
    OCR1B = ocr1b;
    
    TIMSK1 |= (1 << OCIE1B);
    
    // clear in data
    rx_data = 0;
    uart_status &= ~((1<<UART_STATUS_ACTIVE)|(1<<UART_STATUS_RX_DONE));
    bit_count = 0;
}

void tim1_compb_func(){
    uint8_t rx_data_tmp = rx_data;
    uint8_t bit_count_tmp = bit_count;
    // shift to make space for new bit
    rx_data_tmp >>= 1;
    
    // set vaule 1 if pin is hight
    if (SUART_RXD_PIN & (1 << SUART_RXD_BIT)){
        //rx_data_tmp+=1;
        rx_data_tmp |= (1 << 7);
        SUART_TXD_PORT |=  (1 << PA0);
    }
    else
        //  toggle lede to show activity (debug)
        SUART_TXD_PORT &=  ~(1 << PA0);

    bit_count_tmp+=1;
   
    if (bit_count_tmp == 9){
        // disable Combare Match A and enable RX interupts
        TIMSK1 &= ~(1 << OCIE1B);
        //if(rx_data_tmp > 4)
        //    SUART_TXD_PORT |=  (1 << PA0);
        //else
        SUART_TXD_PORT &=  ~(1 << PA0);
        // set uart_status
        uart_status &= ~(1 << UART_STATUS_RX_DONE);
        uart_status |= (1 << UART_STATUS_RX_DONE);
        // enable rx interrupts
        GIMSK  |=  (1 << PCIE0);
        GIFR = (1<<PCIF0);
    } 
    rx_data = rx_data_tmp;
    bit_count = bit_count_tmp;
}

//#ifdef _FIFO_H_
//int uart_getc_wait()
//{
//    return (int) fifo_get_wait (&infifo);
//}
//
//int uart_getc_nowait()
//    return fifo_get_nowait (&infifo);
//}
//
//{
//#else // _FIFO_H_

uint8_t uart_get_wait(){
    while( uart_status & (~(1<<UART_STATUS_RX_DONE)) ){}
    uart_status &= ~(1<<UART_STATUS_RX_DONE);

    return (uint8_t)rx_data;
}

uint8_t uart_get(){
    if(uart_status & (1<<UART_STATUS_RX_DONE)) 
        return (uint8_t)rx_data;
    else
        return 0;
}

//#endif // _FIFO_H


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

    return rx_data;
}

uint8_t uart_get(){
    if(uart_status & (1<<UART_STATUS_RX_DONE)){
        uart_status &= ~(1<<UART_STATUS_RX_DONE);
        return rx_data;
    }
    else
        return 0;
}

//#endif // _FIFO_H


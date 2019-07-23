/* https://rn-wissen.de/wiki/index.php?title=FIFO_mit_avr-gcc 
 * taken from rn-wissen.de
 * (c) unknown?
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#ifdef USE_FIFO
#include "fifo.h"
#endif

#ifndef BAUDRATE
#define BAUDRATE 1024L
#endif

#ifndef INBUF_SIZE
#define INBUF_SIZE 4
#endif

#define SUART_TXD_PORT PORTA
#define SUART_TXD_DDR  DDRA
#define SUART_TXD_BIT  PA1

#define SUART_RXD_PORT PORTA
#define SUART_RXD_PIN  PINA
#define SUART_RXD_DDR  DDRA
#define SUART_RXD_BIT  PA2

#define UART_STATUS_RX_DONE    5
#define UART_STATUS_TX_DONE    6
#define UART_STATUS_COLLISION  7
#define UART_STATUS_ACTIVE     8

#define nop() __asm volatile ("nop")

extern volatile uint16_t tx_data, tx_data_ref;
extern volatile uint8_t bit_count, rx_data;
extern volatile uint8_t uart_status;

#ifdef USE_FIFO
typedef struct
{
    uint8_t volatile count;       // # Zeichen im Puffer
    uint8_t size;                 // Puffer-Größe
    uint8_t *pread;               // Lesezeiger
    uint8_t *pwrite;              // Schreibzeiger
    uint8_t read2end, write2end;  // # Zeichen bis zum Überlauf Lese-/Schreibzeiger
} fifo_t;

//extern void fifo_init (fifo_t*, uint8_t* buf, const uint8_t size);
//extern uint8_t fifo_put (fifo_t*, const uint8_t data);
//extern uint8_t fifo_get_wait (fifo_t*);
//extern int fifo_get_nowait (fifo_t*);

static inline uint8_t
_inline_fifo_put (fifo_t *f, const uint8_t data)
{
    if (f->count >= f->size)
        return 0;
        
    uint8_t * pwrite = f->pwrite;
    
    *(pwrite++) = data;
    
    uint8_t write2end = f->write2end;
    
    if (--write2end == 0)
    {
        write2end = f->size;
        pwrite -= write2end;
    }
    
    f->write2end = write2end;
    f->pwrite = pwrite;

    uint8_t sreg = SREG;
    cli();
    f->count++;
    SREG = sreg;
    
    return 1;
}

static inline uint8_t 
_inline_fifo_get (fifo_t *f)
{
    uint8_t *pread = f->pread;
    uint8_t data = *(pread++);
    uint8_t read2end = f->read2end;
    
    if (--read2end == 0)
    {
        read2end = f->size;
        pread -= read2end;
    }
    
    f->pread = pread;
    f->read2end = read2end;
    
    uint8_t sreg = SREG;
    cli();
    f->count--;
    SREG = sreg;
    
    return data;
}

#endif // USE_FIFO
// send each bit
static inline void tim1_compa_func(){
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
static inline void pcint0_func()
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

static inline void tim1_compb_func(){
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

    bit_count_tmp++;
   
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

extern void uart_init();
extern void uart_putc(uint8_t c);
extern uint8_t uart_get();
extern uint8_t uart_get_wait();
#endif // __UART_H__


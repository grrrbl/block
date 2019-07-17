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

void uart_init();
void uart_putc(uint8_t c);
void tim1_compa_func();
void tim1_compb_func();
void pcint0_func();
uint8_t uart_get();
uint8_t uart_get_wait();
#endif // __UART_H__


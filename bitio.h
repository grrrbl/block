/**
 *  BitIO.h
 *  @author     Andi Dittrich <http://andidittrich.de>
 *  @version    1.0
 *  @license    MIT Style X11 License
*/

#include <stdint.h>

#ifndef __PINIO_H__
#define __PINIO_H__

static inline void PIN_SET(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void PIN_CLEAR(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void PIN_TOGGLE(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void PIN_POSITIVE_EDGE(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
#endif

/*
    PORTA |= (1<<PB1); // ein
    PORTA &= ~(1<<PB1); // aus
    PORTA ^= (1<<PB0); // toggle
*/


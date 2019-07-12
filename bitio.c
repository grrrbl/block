/**
 *  BitIO.h
 *  @author     Andi Dittrich <http://andidittrich.de>
 *  @version    1.0
 *  @license    MIT Style X11 License
*/

#include <stdint.h>

#ifndef __BITIO_H__
#define __BITIO_H__

static inline void PIN_SET(volatile uint8_t *target, uint8_t bit){
    *target |= (1<<bit);
};

static inline void PIN_CLEAR(volatile uint8_t *target, uint8_t bit){
    *target &= ~(1<<bit);
};

static inline void PIN_TOGGLE(volatile uint8_t *target, uint8_t bit){
    *target ^= (1<<bit);

static inline void PIN_POSITIVE_EDGE(volatile uint8_t *target, uint8_t bit){
    *target |= (1<<bit);
    *target &= ~(1<<bit);
};

#endif


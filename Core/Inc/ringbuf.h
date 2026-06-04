#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

#define RINGBUF_SIZE 256

typedef struct {
    uint8_t buf[RINGBUF_SIZE];
    volatile uint16_t head;  /* ISR writes */
    uint16_t tail;           /* main loop reads */
} RingBuf_t;

void RingBuf_Init(RingBuf_t *rb);
bool RingBuf_Put(RingBuf_t *rb, uint8_t byte);     /* ISR-safe, single producer */
bool RingBuf_Get(RingBuf_t *rb, uint8_t *byte);     /* main loop consumer */
uint16_t RingBuf_Available(RingBuf_t *rb);
void RingBuf_Flush(RingBuf_t *rb);

#endif

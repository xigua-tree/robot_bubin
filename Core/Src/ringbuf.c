#include "ringbuf.h"

void RingBuf_Init(RingBuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuf_Put(RingBuf_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % RINGBUF_SIZE;
    if (next == rb->tail) {
        return false;  /* full */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

bool RingBuf_Get(RingBuf_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) {
        return false;  /* empty */
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % RINGBUF_SIZE;
    return true;
}

uint16_t RingBuf_Available(RingBuf_t *rb)
{
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return RINGBUF_SIZE - rb->tail + rb->head;
}

void RingBuf_Flush(RingBuf_t *rb)
{
    rb->tail = rb->head;
}

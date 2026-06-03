#ifndef __RINGBUF_H__
#define __RINGBUF_H__

#include <stdint.h>

#define RINGBUF_SIZE  128

void RingBuf_Init(void);
void RingBuf_PutChar(uint8_t c);
int  RingBuf_GetLine(char *buf, uint32_t timeout_ms);

#endif /* __RINGBUF_H__ */

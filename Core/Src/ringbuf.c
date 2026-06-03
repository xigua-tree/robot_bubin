#include "ringbuf.h"
#include "cmsis_os.h"

static volatile uint8_t  rb_buffer[RINGBUF_SIZE];
static volatile uint32_t rb_head  = 0;   /* ISR writes here */
static volatile uint32_t rb_tail  = 0;   /* Task reads from here */
static volatile uint32_t rb_count = 0;   /* Bytes available */

void RingBuf_Init(void)
{
    rb_head  = 0;
    rb_tail  = 0;
    rb_count = 0;
}

void RingBuf_PutChar(uint8_t c)
{
    if (rb_count < RINGBUF_SIZE) {
        rb_buffer[rb_head] = c;
        rb_head = (rb_head + 1) % RINGBUF_SIZE;
        rb_count++;
    } else {
        /* Buffer full: silently drop oldest byte */
        rb_tail = (rb_tail + 1) % RINGBUF_SIZE;
        rb_buffer[rb_head] = c;
        rb_head = (rb_head + 1) % RINGBUF_SIZE;
        /* rb_count unchanged (dropped one, added one) */
    }
}

int RingBuf_GetLine(char *buf, uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();
    int      idx   = 0;

    while (1) {
        /* Drain available bytes into buf until '\n' or buf full */
        while (rb_count > 0 && idx < 31) {
            char c = (char)rb_buffer[rb_tail];
            rb_tail = (rb_tail + 1) % RINGBUF_SIZE;
            rb_count--;

            if (c == '\n') {
                buf[idx] = '\0';
                return 1;
            }
            if (c != '\r') {
                buf[idx++] = c;
            }
        }

        /* Timeout check */
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            buf[idx] = '\0';
            return (idx > 0) ? 1 : 0;
        }

        osDelay(5);   /* Yield to other tasks */
    }
}

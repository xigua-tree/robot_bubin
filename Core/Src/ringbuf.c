#include "ringbuf.h"
#include "cmsis_os.h"

#define RB_SIZE   RINGBUF_SIZE
#define RB_MASK   (RB_SIZE - 1)   /* SIZE must be power of 2 */

static volatile uint8_t  rb_buffer[RB_SIZE];
static volatile uint32_t rb_head = 0;   /* ISR writes (producer index) */
static volatile uint32_t rb_tail = 0;   /* Task writes, ISR reads */

void RingBuf_Init(void)
{
    rb_head = 0;
    rb_tail = 0;
}

void RingBuf_PutChar(uint8_t c)
{
    uint32_t next_head = (rb_head + 1) & RB_MASK;

    if (next_head != rb_tail) {
        /* Not full: write and advance head */
        rb_buffer[rb_head] = c;
        rb_head = next_head;
    }
    /* else: full — silently drop byte (head unchanged) */
}

int RingBuf_GetLine(char *buf, uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();
    int      idx   = 0;

    while (1) {
        while (rb_head != rb_tail && idx < 31) {
            char c = (char)rb_buffer[rb_tail];
            rb_tail = (rb_tail + 1) & RB_MASK;

            if (c == '\r') {
                /* Consume following '\n' if present */
                if (rb_head != rb_tail && rb_buffer[rb_tail] == '\n') {
                    rb_tail = (rb_tail + 1) & RB_MASK;
                }
                buf[idx] = '\0';
                return 1;
            }
            if (c == '\n') {
                buf[idx] = '\0';
                return 1;
            }
            buf[idx++] = c;
        }

        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            buf[idx] = '\0';
            return (idx > 0) ? 1 : 0;
        }

        osDelay(5);
    }
}

int RingBuf_GetByte(uint8_t *c, uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();

    while (rb_head == rb_tail) {
        /* Empty */
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            return 0;
        }
        osDelay(1);
    }
    *c = rb_buffer[rb_tail];
    rb_tail = (rb_tail + 1) & RB_MASK;
    return 1;
}

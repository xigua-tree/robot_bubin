# Protocol Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Refactor serial protocol from text-based (`M1+50\r\n`) to binary frame format (`CC 33 CMD LEN DATA... DD`) with protocol layer separation.

**Architecture:** New `protocol.c/h` (pure C, no HAL/FreeRTOS deps) handles frame pack/parse. `ringbuf.c/h` gets `RingBuf_GetByte()`. `serial_cmd.c` rewritten with a 6-state frame receive state machine. `motor.h` unchanged.

**Tech Stack:** STM32F407 + STM32 HAL + FreeRTOS (CMSIS-RTOS2) + PlatformIO (ARM GCC 7.2.1)

---

### Task 1: Add RingBuf_GetByte to ringbuf

**Files:**
- Modify: `Core/Inc/ringbuf.h`
- Modify: `Core/Src/ringbuf.c`

- [ ] **Step 1: Add declaration to ringbuf.h**

Insert after the existing `RingBuf_GetLine` declaration:

```c
int RingBuf_GetByte(uint8_t *c, uint32_t timeout_ms);
```

- [ ] **Step 2: Add implementation to ringbuf.c**

Insert before the final line of ringbuf.c (after RingBuf_GetLine):

```c
int RingBuf_GetByte(uint8_t *c, uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();

    while (rb_count == 0) {
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            return 0;   /* timeout */
        }
        osDelay(1);
    }
    *c = rb_buffer[rb_tail];
    rb_tail = (rb_tail + 1) % RINGBUF_SIZE;
    rb_count--;
    return 1;
}
```

- [ ] **Step 3: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: SUCCESS

---

### Task 2: Create protocol.h

**Files:**
- Create: `Core/Inc/protocol.h`

- [ ] **Step 1: Write protocol.h**

```c
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

/* Frame constants */
#define PROTO_HEAD1       0xCC
#define PROTO_HEAD2       0x33
#define PROTO_TAIL        0xDD
#define MAX_DATA_LEN      32
#define FRAME_BUF_SIZE    (5 + MAX_DATA_LEN)

/* Command codes */
#define CMD_SET_DUTY      0x01
#define CMD_GET_ENCODER   0x02
#define CMD_ESTOP         0x03
#define CMD_GET_ENC_ALL   0x04
#define CMD_ERROR         0xFF

/* Error codes */
#define ERR_UNKNOWN_CMD   0x01
#define ERR_LEN_MISMATCH  0x02
#define ERR_INVALID_ID    0x03

/* Pack functions: return total frame length */
int proto_pack_set_duty(uint8_t *buf, uint8_t motor_id, int8_t duty);
int proto_pack_duty_ack(uint8_t *buf);
int proto_pack_get_encoder_req(uint8_t *buf, uint8_t motor_id);
int proto_pack_get_encoder_resp(uint8_t *buf, int32_t enc_val);
int proto_pack_estop_req(uint8_t *buf);
int proto_pack_estop_ack(uint8_t *buf);
int proto_pack_get_enc_all_req(uint8_t *buf);
int proto_pack_get_enc_all_resp(uint8_t *buf, const int32_t enc_vals[4]);
int proto_pack_error(uint8_t *buf, uint8_t errcode);

/* Getters: extract fields from a validated frame (frame points to CMD at offset 2) */
uint8_t proto_get_cmd(const uint8_t *frame);
uint8_t proto_get_len(const uint8_t *frame);
const uint8_t *proto_get_data(const uint8_t *frame);

/* Validation: checks HEAD1/HEAD2 at [0]/[1] and TAIL at [4+LEN] */
int proto_validate_frame(const uint8_t *buf, int buf_len);

#endif /* __PROTOCOL_H__ */
```

---

### Task 3: Create protocol.c

**Files:**
- Create: `Core/Src/protocol.c`

- [ ] **Step 1: Write protocol.c**

```c
#include "protocol.h"
#include <string.h>

/* ============ Internal helpers ============ */

static void write_header(uint8_t *buf, uint8_t cmd, uint8_t len)
{
    buf[0] = PROTO_HEAD1;
    buf[1] = PROTO_HEAD2;
    buf[2] = cmd;
    buf[3] = len;
}

static void write_tail(uint8_t *buf, uint8_t data_len)
{
    buf[4 + data_len] = PROTO_TAIL;
}

static void write_int32_le(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static int32_t read_int32_le(const uint8_t *buf)
{
    return (int32_t)(buf[0])
         | ((int32_t)buf[1] << 8)
         | ((int32_t)buf[2] << 16)
         | ((int32_t)buf[3] << 24);
}

/* ============ Pack functions ============ */

int proto_pack_set_duty(uint8_t *buf, uint8_t motor_id, int8_t duty)
{
    write_header(buf, CMD_SET_DUTY, 2);
    buf[4] = motor_id;
    buf[5] = (uint8_t)duty;
    write_tail(buf, 2);
    return 7;
}

int proto_pack_duty_ack(uint8_t *buf)
{
    write_header(buf, CMD_SET_DUTY, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_encoder_req(uint8_t *buf, uint8_t motor_id)
{
    write_header(buf, CMD_GET_ENCODER, 1);
    buf[4] = motor_id;
    write_tail(buf, 1);
    return 6;
}

int proto_pack_get_encoder_resp(uint8_t *buf, int32_t enc_val)
{
    write_header(buf, CMD_GET_ENCODER, 4);
    write_int32_le(&buf[4], enc_val);
    write_tail(buf, 4);
    return 9;
}

int proto_pack_estop_req(uint8_t *buf)
{
    write_header(buf, CMD_ESTOP, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_estop_ack(uint8_t *buf)
{
    write_header(buf, CMD_ESTOP, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_enc_all_req(uint8_t *buf)
{
    write_header(buf, CMD_GET_ENC_ALL, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_enc_all_resp(uint8_t *buf, const int32_t enc_vals[4])
{
    write_header(buf, CMD_GET_ENC_ALL, 16);
    for (int i = 0; i < 4; i++) {
        write_int32_le(&buf[4 + i * 4], enc_vals[i]);
    }
    write_tail(buf, 16);
    return 21;
}

int proto_pack_error(uint8_t *buf, uint8_t errcode)
{
    write_header(buf, CMD_ERROR, 1);
    buf[4] = errcode;
    write_tail(buf, 1);
    return 6;
}

/* ============ Getters ============ */

uint8_t proto_get_cmd(const uint8_t *frame)
{
    return frame[0];
}

uint8_t proto_get_len(const uint8_t *frame)
{
    return frame[1];
}

const uint8_t *proto_get_data(const uint8_t *frame)
{
    return &frame[2];
}

/* ============ Validation ============ */

int proto_validate_frame(const uint8_t *buf, int buf_len)
{
    if (buf_len < 5) return 0;
    if (buf[0] != PROTO_HEAD1) return 0;
    if (buf[1] != PROTO_HEAD2) return 0;
    uint8_t len = buf[3];
    if (len > MAX_DATA_LEN) return 0;
    if (buf_len < (int)(5 + len)) return 0;
    if (buf[4 + len] != PROTO_TAIL) return 0;
    return 1;
}
```

- [ ] **Step 2: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: protocol.c compiles, project links successfully.

---

### Task 4: Rewrite serial_cmd.c with binary frame state machine

**Files:**
- Rewrite: `Core/Src/serial_cmd.c`

This is the core change. The old text parser is completely replaced.

- [ ] **Step 1: Write the new serial_cmd.c**

```c
#include "serial_cmd.h"
#include "ringbuf.h"
#include "motor.h"
#include "protocol.h"
#include "usart.h"
#include "cmsis_os.h"

/* ============ UART TX using direct register write ============ */
/* Bypasses HAL lock to avoid blocking RX re-arm in ISR callback */

static void UART_Send(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        while (!(huart3.Instance->SR & (1U << 7))) {  /* wait TXE */
            osDelay(1);
        }
        huart3.Instance->DR = data[i];
    }
}
/* ============ Frame receive state machine ============ */

typedef enum {
    STATE_WAIT_H1 = 0,
    STATE_WAIT_H2,
    STATE_GET_CMD,
    STATE_GET_LEN,
    STATE_GET_DATA,
    STATE_GET_TAIL
} FrameState_t;

/* ============ Request handler ============ */

static void HandleRequest(uint8_t *frame, int frame_len)
{
    (void)frame_len;

    uint8_t cmd  = proto_get_cmd(&frame[2]);   /* CMD at offset 2 */
    uint8_t len  = proto_get_len(&frame[2]);   /* LEN at offset 3 */
    const uint8_t *data = proto_get_data(&frame[2]); /* DATA at offset 4 */

    uint8_t resp_buf[FRAME_BUF_SIZE];
    int     resp_len = 0;

    switch (cmd) {

    case CMD_SET_DUTY:
        if (len != 2) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            uint8_t motor_id = data[0];
            int8_t  duty     = (int8_t)data[1];
            if (motor_id >= 4) {
                resp_len = proto_pack_error(resp_buf, ERR_INVALID_ID);
                break;
            }
            Motor_SetDuty((Motor_ID_t)motor_id, duty);
            resp_len = proto_pack_duty_ack(resp_buf);
        }
        break;

    case CMD_GET_ENCODER:
        if (len != 1) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            uint8_t motor_id = data[0];
            if (motor_id >= 4) {
                resp_len = proto_pack_error(resp_buf, ERR_INVALID_ID);
                break;
            }
            int32_t enc = Motor_GetEncoder((Motor_ID_t)motor_id);
            resp_len = proto_pack_get_encoder_resp(resp_buf, enc);
        }
        break;

    case CMD_ESTOP:
        if (len != 0) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        Motor_StopAll();
        resp_len = proto_pack_estop_ack(resp_buf);
        break;

    case CMD_GET_ENC_ALL:
        if (len != 0) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            int32_t enc_vals[4];
            enc_vals[0] = Motor_GetEncoder(MOTOR_1);
            enc_vals[1] = Motor_GetEncoder(MOTOR_2);
            enc_vals[2] = Motor_GetEncoder(MOTOR_3);
            enc_vals[3] = Motor_GetEncoder(MOTOR_4);
            resp_len = proto_pack_get_enc_all_resp(resp_buf, enc_vals);
        }
        break;

    default:
        resp_len = proto_pack_error(resp_buf, ERR_UNKNOWN_CMD);
        break;
    }

    if (resp_len > 0) {
        UART_Send(resp_buf, resp_len);
    }
}

/* ============ FreeRTOS task ============ */

void SerialCmdTask(void *argument)
{
    (void)argument;

    uint8_t frame_buf[FRAME_BUF_SIZE];
    int     frame_idx  = 0;
    FrameState_t state = STATE_WAIT_H1;
    uint8_t data_len   = 0;
    int     data_count = 0;

    for (;;) {
        uint8_t c;
        if (!RingBuf_GetByte(&c, 100)) {
            /* Timeout: reset to start of frame */
            state = STATE_WAIT_H1;
            continue;
        }

        switch (state) {

        case STATE_WAIT_H1:
            if (c == PROTO_HEAD1) {
                frame_buf[0] = c;
                frame_idx = 1;
                state = STATE_WAIT_H2;
            }
            /* else: stay in WAIT_H1, consume byte */
            break;

        case STATE_WAIT_H2:
            if (c == PROTO_HEAD1) {
                /* Another HEAD1 → stay in H2 (re-sync) */
                frame_buf[0] = c;
                frame_idx = 1;
            } else if (c == PROTO_HEAD2) {
                frame_buf[1] = c;
                frame_idx = 2;
                state = STATE_GET_CMD;
            } else {
                /* Wrong byte, go back to hunting */
                state = STATE_WAIT_H1;
            }
            break;

        case STATE_GET_CMD:
            frame_buf[2] = c;
            frame_idx = 3;
            state = STATE_GET_LEN;
            break;

        case STATE_GET_LEN:
            data_len = c;
            if (data_len > MAX_DATA_LEN) {
                /* Bogus length, discard */
                state = STATE_WAIT_H1;
                break;
            }
            frame_buf[3] = c;
            frame_idx = 4;
            data_count = 0;
            state = (data_len > 0) ? STATE_GET_DATA : STATE_GET_TAIL;
            break;

        case STATE_GET_DATA:
            frame_buf[4 + data_count] = c;
            data_count++;
            frame_idx++;
            if (data_count >= data_len) {
                state = STATE_GET_TAIL;
            }
            break;

        case STATE_GET_TAIL:
            if (c == PROTO_TAIL) {
                frame_buf[4 + data_len] = c;
                int total_len = 5 + data_len;
                HandleRequest(frame_buf, total_len);
            }
            /* Frame tail mismatch → silently discard */
            state = STATE_WAIT_H1;
            break;
        }
    }
}
```

- [ ] **Step 2: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: serial_cmd.c compiles, project links successfully.

---

### Task 5: Build and verify

- [ ] **Step 1: Clean rebuild**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407 -t clean && pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: Zero errors, zero warnings.

---

## Summary

| File | Status | Lines |
|------|--------|-------|
| `Core/Inc/protocol.h` | NEW | ~45 |
| `Core/Src/protocol.c` | NEW | ~135 |
| `Core/Inc/ringbuf.h` | MODIFY | +1 line |
| `Core/Src/ringbuf.c` | MODIFY | +18 lines |
| `Core/Src/serial_cmd.c` | REWRITE | ~160 lines |

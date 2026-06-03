#include "serial_cmd.h"
#include "ringbuf.h"
#include "motor.h"
#include "protocol.h"
#include "usart.h"
#include "cmsis_os.h"

extern osThreadId_t led_testHandle;  /* from freertos.c */

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

static void HandleRequest(const uint8_t *frame)
{
    uint8_t       cmd  = proto_get_cmd(&frame[2]);   /* CMD at offset 2 */
    uint8_t       len  = proto_get_len(&frame[2]);   /* LEN at offset 3 */
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

    uint8_t      frame_buf[FRAME_BUF_SIZE];
    FrameState_t state      = STATE_WAIT_H1;
    uint8_t      data_len   = 0;
    int          data_count = 0;

    for (;;) {
        uint8_t c;
        if (!RingBuf_GetByte(&c, 100)) {
            /* Timeout: reset frame parser */
            state = STATE_WAIT_H1;
            continue;
        }

        switch (state) {

        case STATE_WAIT_H1:
            if (c == PROTO_HEAD1) {
                frame_buf[0] = c;
                state = STATE_WAIT_H2;
            }
            break;

        case STATE_WAIT_H2:
            if (c == PROTO_HEAD1) {
                /* Back-to-back HEAD1: stay in H2 with re-sync */
                frame_buf[0] = c;
            } else if (c == PROTO_HEAD2) {
                frame_buf[1] = c;
                state = STATE_GET_CMD;
            } else {
                state = STATE_WAIT_H1;
            }
            break;

        case STATE_GET_CMD:
            frame_buf[2] = c;
            state = STATE_GET_LEN;
            break;

        case STATE_GET_LEN:
            data_len = c;
            if (data_len > MAX_DATA_LEN) {
                state = STATE_WAIT_H1;
                break;
            }
            frame_buf[3] = c;
            data_count = 0;
            state = (data_len > 0) ? STATE_GET_DATA : STATE_GET_TAIL;
            break;

        case STATE_GET_DATA:
            frame_buf[4 + data_count] = c;
            data_count++;
            if (data_count >= data_len) {
                state = STATE_GET_TAIL;
            }
            break;

        case STATE_GET_TAIL:
            if (c == PROTO_TAIL) {
                frame_buf[4 + data_len] = c;
                HandleRequest(frame_buf);
                osThreadFlagsSet(led_testHandle, 0x01);
            }
            /* Mismatch or success: reset to hunt next frame */
            state = STATE_WAIT_H1;
            break;
        }
    }
}

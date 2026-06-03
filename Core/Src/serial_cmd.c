#include "serial_cmd.h"
#include "ringbuf.h"
#include "motor.h"
#include "protocol.h"
#include "speed_ctrl.h"
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

    /* ===== Speed control commands ===== */

    case CMD_SET_MODE:
        if (len != 2) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            uint8_t motor_id = data[0];
            uint8_t mode     = data[1];
            if (motor_id >= 4 || mode > 1) {
                resp_len = proto_pack_error(resp_buf, ERR_INVALID_ID);
                break;
            }
            SpeedCtrl_SetMode((Motor_ID_t)motor_id, mode);
            resp_len = proto_pack_mode_ack(resp_buf);
        }
        break;

    case CMD_SET_RPM:
        if (len != 3) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            uint8_t motor_id = data[0];
            int16_t rpm      = (int16_t)(data[1] | ((int16_t)data[2] << 8));
            if (motor_id >= 4) {
                resp_len = proto_pack_error(resp_buf, ERR_INVALID_ID);
                break;
            }
            /* rpm is ×10: 1234 → 123.4 RPM */
            SpeedCtrl_SetTargetRPM((Motor_ID_t)motor_id, (float)rpm * 0.1f);
            resp_len = proto_pack_rpm_ack(resp_buf);
        }
        break;

    case CMD_SET_PID:
        if (len != 7) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            uint8_t motor_id = data[0];
            int16_t kp_raw   = (int16_t)(data[1] | ((int16_t)data[2] << 8));
            int16_t ki_raw   = (int16_t)(data[3] | ((int16_t)data[4] << 8));
            int16_t kd_raw   = (int16_t)(data[5] | ((int16_t)data[6] << 8));
            if (motor_id >= 4) {
                resp_len = proto_pack_error(resp_buf, ERR_INVALID_ID);
                break;
            }
            /* Coefficients are ×100: 150 → Kp=1.5 */
            float kp = (float)kp_raw * 0.01f;
            float ki = (float)ki_raw * 0.01f;
            float kd = (float)kd_raw * 0.01f;
            SpeedCtrl_SetPID((Motor_ID_t)motor_id, kp, ki, kd);
            resp_len = proto_pack_pid_ack(resp_buf);
        }
        break;

    case CMD_GET_RPM:
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
            float rpm_f = SpeedCtrl_GetCurrentRPM((Motor_ID_t)motor_id);
            int16_t rpm = (int16_t)(rpm_f * 10.0f);  /* ×10 for int16 */
            resp_len = proto_pack_get_rpm_resp(resp_buf, rpm);
        }
        break;

    case CMD_GET_RPM_ALL:
        if (len != 0) {
            resp_len = proto_pack_error(resp_buf, ERR_LEN_MISMATCH);
            break;
        }
        {
            int16_t rpm_vals[4];
            for (int i = 0; i < 4; i++) {
                float rpm_f = SpeedCtrl_GetCurrentRPM((Motor_ID_t)i);
                rpm_vals[i] = (int16_t)(rpm_f * 10.0f);
            }
            resp_len = proto_pack_get_rpm_all_resp(resp_buf, rpm_vals);
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

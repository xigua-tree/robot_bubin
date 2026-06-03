#include "bt_proto.h"
#include "ringbuf.h"
#include "speed_ctrl.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

extern osThreadId_t led_testHandle;  /* from freertos.c */

/* ============ UART TX (direct register, same as before) ============ */

static void UART_Send(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        while (!(huart3.Instance->SR & (1U << 7))) { osDelay(1); }
        huart3.Instance->DR = data[i];
    }
}

/* ============ Checksum ============ */

static uint8_t checksum(const uint8_t *data, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* ============ Build outgoing frame ============ */

static void bt_send_speed(void)
{
    float speed = SpeedCtrl_GetCurrentRPM(MOTOR_1);
    uint8_t buf[BT_TX_LEN];
    buf[0] = BT_HEAD;
    memcpy(&buf[1], &speed, 4);             /* native LE */
    buf[5] = checksum(&buf[1], 4);
    buf[6] = BT_TAIL;
    UART_Send(buf, BT_TX_LEN);
}

/* ============ Incoming frame dispatch ============ */

static void bt_dispatch(const uint8_t *data)
{
    float speed, kp, ki, kd;
    memcpy(&speed, &data[0], 4);
    memcpy(&kp,    &data[4], 4);
    memcpy(&ki,    &data[8], 4);
    memcpy(&kd,    &data[12], 4);

    for (int i = 0; i < 4; i++) {
        SpeedCtrl_SetPID((Motor_ID_t)i, kp, ki, kd);
        SpeedCtrl_SetTargetRPM((Motor_ID_t)i, speed);
    }
}

/* ============ Receive state machine ============ */

typedef enum {
    STATE_WAIT_HEAD = 0,
    STATE_GET_DATA,
    STATE_GET_CS,
    STATE_GET_TAIL
} BtRxState;

/* ============ FreeRTOS task ============ */

void BtHandlerTask(void *argument)
{
    (void)argument;

    uint8_t    rx_buf[BT_RX_LEN];
    int        rx_idx    = 0;
    int        data_cnt  = 0;
    BtRxState  state     = STATE_WAIT_HEAD;
    TickType_t last_tx   = 0;

    for (;;) {
        uint8_t c;
        if (RingBuf_GetByte(&c, 50)) {
            switch (state) {

            case STATE_WAIT_HEAD:
                if (c == BT_HEAD) {
                    rx_buf[0] = c;
                    rx_idx    = 1;
                    data_cnt  = 0;
                    state     = STATE_GET_DATA;
                }
                break;

            case STATE_GET_DATA:
                rx_buf[rx_idx++] = c;
                if (++data_cnt >= 16) state = STATE_GET_CS;
                break;

            case STATE_GET_CS:
                rx_buf[rx_idx++] = c;    /* store CS byte */
                state = STATE_GET_TAIL;
                break;

            case STATE_GET_TAIL:
                rx_buf[rx_idx] = c;
                if (c == BT_TAIL) {
                    uint8_t expected = checksum(&rx_buf[1], 16);
                    if (expected == rx_buf[17]) {
                        bt_dispatch(&rx_buf[1]);
                        osThreadFlagsSet(led_testHandle, 0x01);
                    }
                }
                state = STATE_WAIT_HEAD;
                break;
            }
        } else {
            state = STATE_WAIT_HEAD;   /* timeout: reset parser */
        }

        /* Always send speed every 50ms, even without incoming data */
        TickType_t now = osKernelGetTickCount();
        if ((now - last_tx) >= 100) {
            bt_send_speed();
            last_tx = now;
        }
    }
}

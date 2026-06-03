#include "serial_cmd.h"
#include "ringbuf.h"
#include "motor.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* ============ Helper: send string via UART ============ */
static void UART_SendStr(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 100);
}

/* ============ Command parser ============ */
static int ParseAndExec(const char *cmd)
{
    /* --- STOP (case-insensitive, skips leading whitespace) --- */
    const char *p = cmd;
    while (*p == ' ' || *p == '\t') p++;

    if ((p[0] == 'S' || p[0] == 's') &&
        (p[1] == 'T' || p[1] == 't') &&
        (p[2] == 'O' || p[2] == 'o') &&
        (p[3] == 'P' || p[3] == 'p') &&
        (p[4] == '\0' || p[4] == '\r' || p[4] == '\n' || p[4] == ' ')) {
        Motor_StopAll();
        UART_SendStr("OK\r\n");
        return 0;
    }

    /* --- ENCALL --- */
    if ((p[0] == 'E' || p[0] == 'e') &&
        (p[1] == 'N' || p[1] == 'n') &&
        (p[2] == 'C' || p[2] == 'c') &&
        (p[3] == 'A' || p[3] == 'a') &&
        (p[4] == 'L' || p[4] == 'l') &&
        (p[5] == 'L' || p[5] == 'l')) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "E1:%ld E2:%ld E3:%ld E4:%ld\r\n",
                           (long)Motor_GetEncoder(MOTOR_1),
                           (long)Motor_GetEncoder(MOTOR_2),
                           (long)Motor_GetEncoder(MOTOR_3),
                           (long)Motor_GetEncoder(MOTOR_4));
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100);
        return 0;
    }

    /* --- ENC<id> --- */
    if ((p[0] == 'E' || p[0] == 'e') &&
        (p[1] == 'N' || p[1] == 'n') &&
        (p[2] == 'C' || p[2] == 'c') &&
        p[3] >= '1' && p[3] <= '4') {
        Motor_ID_t id = (Motor_ID_t)(p[3] - '1');
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%ld\r\n",
                           (long)Motor_GetEncoder(id));
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100);
        return 0;
    }

    /* --- M<id><+/-><duty> --- */
    if ((p[0] == 'M' || p[0] == 'm') &&
        p[1] >= '1' && p[1] <= '4') {
        Motor_ID_t id = (Motor_ID_t)(p[1] - '1');

        if (p[2] == '+' || p[2] == '-') {
            /* Parse duty value manually (avoid strtol) */
            int duty = 0;
            const char *q = &p[3];
            while (*q >= '0' && *q <= '9') {
                duty = duty * 10 + (*q - '0');
                q++;
            }
            if (duty > 100) duty = 100;

            if (p[2] == '-') duty = -duty;

            Motor_SetDuty(id, (int8_t)duty);
            UART_SendStr("OK\r\n");
            return 0;
        }
    }

    /* --- Unknown command --- */
    UART_SendStr("ERR\r\n");
    return -1;
}

/* ============ FreeRTOS task function ============ */
void SerialCmdTask(void *argument)
{
    (void)argument;

    char cmd[32];

    for (;;) {
        if (RingBuf_GetLine(cmd, 1000)) {
            ParseAndExec(cmd);
        }
    }
}

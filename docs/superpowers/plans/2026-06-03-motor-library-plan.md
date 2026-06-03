# Motor Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a 4-channel motor library with TB6612 driver support, expose a duty-cycle API, and add serial command control (text-based) in the FreeRTOS environment.

**Architecture:** Three-layer design — `ringbuf` (ISR-safe byte buffer) → `serial_cmd` (text parser + FreeRTOS task) → `motor` (HAL PWM + GPIO direction). Each layer has no knowledge of the layer above it. `motor.c` stores 4 static `Motor_Context_t` structs with direction GPIO, TIM8 PWM channel, and encoder timer references.

**Tech Stack:** STM32F407 + STM32 HAL + FreeRTOS (CMSIS-RTOS2) + PlatformIO (ARM GCC)

**Tick config:** `configTICK_RATE_HZ = 1000` → 1 tick = 1 ms

---

## File Responsibility Map

| File | Status | Responsibility |
|------|--------|---------------|
| `Core/Inc/ringbuf.h` | NEW | Ring buffer types and API declarations |
| `Core/Src/ringbuf.c` | NEW | 128-byte circular buffer, ISR-safe write, blocking read with timeout |
| `Core/Inc/motor.h` | NEW | `Motor_ID_t` enum, public motor API declarations |
| `Core/Src/motor.c` | NEW | 4x `Motor_Context_t` static array, TB6612 control logic, encoder read |
| `Core/Inc/serial_cmd.h` | NEW | `SerialCmdTask` function declaration |
| `Core/Src/serial_cmd.c` | NEW | Command parser (`M1+50`, `STOP`, `ENC1`, `ENCALL`) + FreeRTOS task |
| `Core/Src/main.c` | MODIFY | Call `Motor_InitAll()`, start UART RX, implement `HAL_UART_RxCpltCallback` |
| `Core/Src/freertos.c` | MODIFY | Create `SerialCmdTask` in `MX_FREERTOS_Init` |

---

### Task 1: Create ring buffer header

**Files:**
- Create: `Core/Inc/ringbuf.h`

- [ ] **Step 1: Write ringbuf.h**

```c
#ifndef __RINGBUF_H__
#define __RINGBUF_H__

#include <stdint.h>

#define RINGBUF_SIZE  128

void RingBuf_Init(void);
void RingBuf_PutChar(uint8_t c);
int  RingBuf_GetLine(char *buf, uint32_t timeout_ms);

#endif /* __RINGBUF_H__ */
```

---

### Task 2: Create ring buffer implementation

**Files:**
- Create: `Core/Src/ringbuf.c`

Dependencies: Task 1 (`ringbuf.h`)

- [ ] **Step 1: Write ringbuf.c**

```c
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
```

- [ ] **Step 2: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: Compilation passes (ringbuf.c compiles, no linker errors about undefined symbols yet since we haven't added to main)

---

### Task 3: Create motor library header

**Files:**
- Create: `Core/Inc/motor.h`

- [ ] **Step 1: Write motor.h**

```c
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_MAX = 4
} Motor_ID_t;

/* Initialization */
void Motor_InitAll(void);
void Motor_DeInitAll(void);

/* Core control: duty range [-100, 100], returns 0=OK, -1=bad ID */
int  Motor_SetDuty(Motor_ID_t id, int8_t duty);
void Motor_Stop(Motor_ID_t id);
void Motor_StopAll(void);

/* Encoder */
int32_t Motor_GetEncoder(Motor_ID_t id);
void    Motor_ResetEncoder(Motor_ID_t id);

#endif /* __MOTOR_H__ */
```

---

### Task 4: Create motor library implementation

**Files:**
- Create: `Core/Src/motor.c`

Dependencies: Task 3 (`motor.h`), `tim.h` (existing — declares `htim1`-`htim4`, `htim8`)

- [ ] **Step 1: Write motor.c**

```c
#include "motor.h"
#include "tim.h"

/* ============ Internal motor context ============ */
typedef struct {
    Motor_ID_t          id;
    TIM_HandleTypeDef  *htim;           /* &htim8 for PWM */
    uint32_t            tim_channel;    /* TIM_CHANNEL_1 ~ TIM_CHANNEL_4 */
    GPIO_TypeDef       *dir1_port;      /* TB6612 AIN1 — direction GPIO */
    uint16_t            dir1_pin;
    GPIO_TypeDef       *dir2_port;      /* TB6612 AIN2 — direction GPIO */
    uint16_t            dir2_pin;
    TIM_HandleTypeDef  *encoder_htim;   /* &htim1 ~ &htim4 */
} Motor_Context_t;

static Motor_Context_t g_motors[MOTOR_MAX];

/* ============ Public API ============ */

void Motor_InitAll(void)
{
    /* --- Motor 1: PE14→IN1, PE15→IN2, TIM8_CH1→PWMA, TIM1 encoder --- */
    g_motors[MOTOR_1].id            = MOTOR_1;
    g_motors[MOTOR_1].htim          = &htim8;
    g_motors[MOTOR_1].tim_channel   = TIM_CHANNEL_1;
    g_motors[MOTOR_1].dir1_port     = GPIOE;
    g_motors[MOTOR_1].dir1_pin      = GPIO_PIN_14;
    g_motors[MOTOR_1].dir2_port     = GPIOE;
    g_motors[MOTOR_1].dir2_pin      = GPIO_PIN_15;
    g_motors[MOTOR_1].encoder_htim  = &htim1;

    /* --- Motor 2: PE0→IN1, PE1→IN2, TIM8_CH3→PWMA, TIM2 encoder --- */
    g_motors[MOTOR_2].id            = MOTOR_2;
    g_motors[MOTOR_2].htim          = &htim8;
    g_motors[MOTOR_2].tim_channel   = TIM_CHANNEL_3;
    g_motors[MOTOR_2].dir1_port     = GPIOE;
    g_motors[MOTOR_2].dir1_pin      = GPIO_PIN_0;
    g_motors[MOTOR_2].dir2_port     = GPIOE;
    g_motors[MOTOR_2].dir2_pin      = GPIO_PIN_1;
    g_motors[MOTOR_2].encoder_htim  = &htim2;

    /* --- Motor 3: PE12→IN1, PE13→IN2, TIM8_CH2→PWMA, TIM3 encoder --- */
    g_motors[MOTOR_3].id            = MOTOR_3;
    g_motors[MOTOR_3].htim          = &htim8;
    g_motors[MOTOR_3].tim_channel   = TIM_CHANNEL_2;
    g_motors[MOTOR_3].dir1_port     = GPIOE;
    g_motors[MOTOR_3].dir1_pin      = GPIO_PIN_12;
    g_motors[MOTOR_3].dir2_port     = GPIOE;
    g_motors[MOTOR_3].dir2_pin      = GPIO_PIN_13;
    g_motors[MOTOR_3].encoder_htim  = &htim3;

    /* --- Motor 4: PB8→IN1, PB9→IN2, TIM8_CH4→PWMA, TIM4 encoder --- */
    g_motors[MOTOR_4].id            = MOTOR_4;
    g_motors[MOTOR_4].htim          = &htim8;
    g_motors[MOTOR_4].tim_channel   = TIM_CHANNEL_4;
    g_motors[MOTOR_4].dir1_port     = GPIOB;
    g_motors[MOTOR_4].dir1_pin      = GPIO_PIN_8;
    g_motors[MOTOR_4].dir2_port     = GPIOB;
    g_motors[MOTOR_4].dir2_pin      = GPIO_PIN_9;
    g_motors[MOTOR_4].encoder_htim  = &htim4;

    /* Start PWM on all 4 TIM8 channels */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* All motors stopped */
    Motor_StopAll();
}

void Motor_DeInitAll(void)
{
    Motor_StopAll();
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
}

int Motor_SetDuty(Motor_ID_t id, int8_t duty)
{
    if (id >= MOTOR_MAX) {
        return -1;
    }

    /* Clamp to valid range */
    if (duty > 100)  duty = 100;
    if (duty < -100) duty = -100;

    Motor_Context_t *m   = &g_motors[id];
    uint32_t         arr = m->htim->Init.Period;   /* 16799 */
    uint32_t         pulse;

    if (duty > 0) {
        /* Forward: IN1=H, IN2=L, PWMA=PWM */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
        pulse = (arr + 1) * (uint32_t)duty / 100;
    } else if (duty < 0) {
        /* Reverse: IN1=L, IN2=H, PWMA=PWM */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_SET);
        pulse = (arr + 1) * (uint32_t)(-duty) / 100;
    } else {
        /* Coast stop: IN1=L, IN2=L, PWMA=0 */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, pulse);
    return 0;
}

void Motor_Stop(Motor_ID_t id)
{
    Motor_SetDuty(id, 0);
}

void Motor_StopAll(void)
{
    for (int i = MOTOR_1; i < MOTOR_MAX; i++) {
        Motor_Stop((Motor_ID_t)i);
    }
}

int32_t Motor_GetEncoder(Motor_ID_t id)
{
    if (id >= MOTOR_MAX) {
        return 0;
    }
    return (int32_t)__HAL_TIM_GET_COUNTER(g_motors[id].encoder_htim);
}

void Motor_ResetEncoder(Motor_ID_t id)
{
    if (id >= MOTOR_MAX) {
        return;
    }
    __HAL_TIM_SET_COUNTER(g_motors[id].encoder_htim, 0);
}
```

- [ ] **Step 2: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: motor.c compiles successfully along with ringbuf.c.

---

### Task 5: Create serial command header

**Files:**
- Create: `Core/Inc/serial_cmd.h`

- [ ] **Step 1: Write serial_cmd.h**

```c
#ifndef __SERIAL_CMD_H__
#define __SERIAL_CMD_H__

void SerialCmdTask(void *argument);

#endif /* __SERIAL_CMD_H__ */
```

---

### Task 6: Create serial command implementation

**Files:**
- Create: `Core/Src/serial_cmd.c`

Dependencies: Tasks 1-5

- [ ] **Step 1: Write serial_cmd.c**

```c
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
```

- [ ] **Step 2: Verify compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: All 3 new .c files compile successfully. No linker errors.

---

### Task 7: Modify main.c — initialize motors and start UART RX

**Files:**
- Modify: `Core/Src/main.c`

Dependencies: Tasks 1-6 all compile successfully

- [ ] **Step 1: Add includes in USER CODE BEGIN Includes (around line 28)**

Add after `/* USER CODE BEGIN Includes */`:

```c
#include "motor.h"
#include "ringbuf.h"
```

- [ ] **Step 2: Add rx_byte variable in USER CODE BEGIN PV (around line 48)**

Add after `/* USER CODE BEGIN PV */`:

```c
static volatile uint8_t rx_byte;
```

- [ ] **Step 3: Add initialization in USER CODE BEGIN 2 (around line 100)**

Add after `/* USER CODE BEGIN 2 */`:

```c
  Motor_InitAll();
  RingBuf_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
```

Note: `rx_byte` is const-qualified in the scope, but `HAL_UART_Receive_IT` takes `uint8_t *`. Cast away volatile with `(uint8_t *)&rx_byte` — the HAL callback will write to it in interrupt context.

- [ ] **Step 4: Add HAL_UART_RxCpltCallback in USER CODE BEGIN 4 (around line 168)**

Add after `/* USER CODE BEGIN 4 */`:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        RingBuf_PutChar(rx_byte);
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
    }
}
```

- [ ] **Step 5: Verify full compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: Full project compiles and links successfully.

---

### Task 8: Modify freertos.c — create SerialCmdTask

**Files:**
- Modify: `Core/Src/freertos.c`

Dependencies: Task 7 passes

- [ ] **Step 1: Add include in USER CODE BEGIN Includes (around line 28)**

Add after `/* USER CODE BEGIN Includes */`:

```c
#include "serial_cmd.h"
```

- [ ] **Step 2: Add task attributes in USER CODE BEGIN Variables (around line 48)**

Add after `/* USER CODE BEGIN Variables */`:

```c
osThreadId_t serialCmdTaskHandle;
const osThreadAttr_t serialCmdTask_attributes = {
  .name = "SerialCmdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
```

- [ ] **Step 3: Create task in USER CODE BEGIN RTOS_THREADS (around line 107)**

Add after `/* USER CODE BEGIN RTOS_THREADS */`:

```c
  serialCmdTaskHandle = osThreadNew(SerialCmdTask, NULL, &serialCmdTask_attributes);
```

- [ ] **Step 4: Verify full compilation**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: Full project compiles and links successfully.

---

### Task 9: Build and verify

**Files:** None new — verification step

- [ ] **Step 1: Clean rebuild**

Run: `pio run -d e:/My_Learn_for_freertos/test_for_f407 -t clean && pio run -d e:/My_Learn_for_freertos/test_for_f407`
Expected: Clean build passes with zero errors and zero warnings.

- [ ] **Step 2: Check binary size**

Run: `ls -la e:/My_Learn_for_freertos/test_for_f407/.pio/build/genericSTM32F407VET6/firmware.elf`
Expected: ELF file exists and is a reasonable size.

- [ ] **Step 3: Check all new files exist**

Run: `ls -la e:/My_Learn_for_freertos/test_for_f407/Core/Inc/{ringbuf,motor,serial_cmd}.h e:/My_Learn_for_freertos/test_for_f407/Core/Src/{ringbuf,motor,serial_cmd}.c`
Expected: All 6 files present.

---

## Summary of All Changes

### New files (6)
| File | Lines |
|------|-------|
| `Core/Inc/ringbuf.h` | 12 |
| `Core/Src/ringbuf.c` | 57 |
| `Core/Inc/motor.h` | 23 |
| `Core/Src/motor.c` | 145 |
| `Core/Inc/serial_cmd.h` | 9 |
| `Core/Src/serial_cmd.c` | 108 |

### Modified files (2)
| File | Change |
|------|--------|
| `Core/Src/main.c` | 4 insertions: 2 includes + 1 variable + 3 init lines + callback function |
| `Core/Src/freertos.c` | 3 insertions: 1 include + task attrs + osThreadNew call |

### PlatformIO config
NO changes needed — `build_src_filter` already includes `+<Core/Src/>`, and `-I Core/Inc` is in `build_flags`.

# Motor Library Design

**Date:** 2026-06-03
**Project:** test_for_f407 (STM32F407 + FreeRTOS + PlatformIO)
**Topic:** 四路带编码器直流电机库 + 串口占空比调试

---

## 1. Hardware Configuration

| Motor | Direction Pins | PWM (TIM8) | Encoder     |
|-------|---------------|------------|-------------|
| 1     | PE14, PE15    | CH1 (PC6)  | TIM1 CH1/CH2 (PA8/PA9) |
| 2     | PE0, PE1      | CH3 (PC8)  | TIM2 CH1/CH2 (PA0/PA1) |
| 3     | PE12, PE13    | CH2 (PC7)  | TIM3 CH1/CH2 (PA6/PA7) |
| 4     | PB8, PB9      | CH4 (PC9)  | TIM4 CH1/CH2 (PB6/PB7) |

- Driver chip: **TB6612** (3-wire control per motor)
  - AIN1 → direction GPIO (IN1)
  - AIN2 → direction GPIO (IN2)
  - PWMA → speed PWM (TIM8_CHx, connected via PC6~PC9)
  - Motor1: PE14→IN1, PE15→IN2, PC6(TIM8_CH1)→PWMA
  - Motor2: PE0→IN1, PE1→IN2, PC8(TIM8_CH3)→PWMA
  - Motor3: PE12→IN1, PE13→IN2, PC7(TIM8_CH2)→PWMA
  - Motor4: PB8→IN1, PB9→IN2, PC9(TIM8_CH4)→PWMA
- PWM: TIM8 CH1~CH4 (PC6~PC9), ARR=16799, Prescaler=0, f≈10kHz
- USART3: 115200 bps, 8N1
  - RX: byte-by-byte via `HAL_UART_Receive_IT` + `HAL_UART_RxCpltCallback` (re-arm after each byte)
- All peripherals already configured and initialized by CubeMX

## 2. File Structure

```
Core/
├── Inc/
│   ├── motor.h          ← [NEW] Motor library public API
│   ├── serial_cmd.h     ← [NEW] Serial command parser public API
│   └── ringbuf.h        ← [NEW] Ring buffer utility
└── Src/
    ├── motor.c          ← [NEW] Motor library implementation
    ├── serial_cmd.c     ← [NEW] Serial command parser implementation
    ├── ringbuf.c        ← [NEW] Ring buffer implementation
    ├── main.c           ← [MODIFY] Add Motor_InitAll() call
    └── freertos.c       ← [MODIFY] Add SerialCmdTask creation
```

## 3. Module Architecture

```
USART3 IRQ Handler (interrupt context)
    │
    │ RingBuf_PutChar()
    ▼
Ring Buffer (128 bytes, ISR-safe write)
    │
    │ RingBuf_GetLine() with timeout
    ▼
SerialCmdTask (FreeRTOS task, priority Normal, stack 256*4)
    │
    │ ParseMotorCmd() → Motor_SetDuty()
    ▼
motor.c / motor.h
    │
    ├── HAL_TIM_PWM_ConfigChannel / HAL_TIM_PWM_Start (TIM8)
    ├── HAL_GPIO_WritePin (PE14/PE15/PE0/PE1/PE12/PE13/PB8/PB9)
    └── __HAL_TIM_GET_COUNTER (encoder read via TIM1~4)
```

**Design principles:**
- `motor.c` provides a clean API — it does not know who calls it (serial, PID, host protocol)
- `serial_cmd.c` only parses UART text and calls motor API — it does not know motor internals
- Ring buffer is a standalone utility with ISR-safe write and blocking read

## 4. Motor Library API

### 4.1 Public Types

```c
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_MAX = 4
} Motor_ID_t;
```

### 4.2 Internal Structure

```c
typedef struct {
    Motor_ID_t          id;
    TIM_HandleTypeDef  *htim;          // &htim8
    uint32_t            tim_channel;   // TIM_CHANNEL_1~4
    GPIO_TypeDef       *dir1_port;     // TB6612 IN1
    uint16_t            dir1_pin;
    GPIO_TypeDef       *dir2_port;     // TB6612 IN2
    uint16_t            dir2_pin;
    TIM_HandleTypeDef  *encoder_htim;  // &htim1~4
} Motor_Context_t;
```

Instances are statically allocated as a file-scope array in `motor.c` (not heap-allocated).

### 4.3 Public Functions

```c
/* Initialization */
void Motor_InitAll(void);
void Motor_DeInitAll(void);

/* Core control */
int  Motor_SetDuty(Motor_ID_t id, int8_t duty);  // -100 ~ +100, returns 0=OK, -1=bad ID
void Motor_Stop(Motor_ID_t id);
void Motor_StopAll(void);

/* Encoder (reserved for future host app) */
int32_t Motor_GetEncoder(Motor_ID_t id);
void    Motor_ResetEncoder(Motor_ID_t id);
```

### 4.4 TB6612 Control Logic

| duty | IN1 (dir1) | IN2 (dir2) | PWMA (TIM8 CCRx) | Effect  |
|------|-----------|------------|-------------------|---------|
| >0   | HIGH      | LOW        | ARR * duty / 100  | Forward |
| <0   | LOW       | HIGH       | ARR * |duty| / 100 | Reverse |
| 0    | LOW       | LOW        | 0                 | Coast   |

- dir1_pin → TB6612 AIN1, dir2_pin → TB6612 AIN2 (pure GPIO, direction only)
- TIM8 CCRx → TB6612 PWMA (PWM speed, duty = 0% ~ 100%)
- PWM duty cycle register value = `(TIM8 ARR + 1) * |duty| / 100`
- Coast stop at duty=0; short-brake (IN1=H,IN2=H) reserved for future use

### 4.5 Concurrency

No mutex required. Each motor occupies independent TIM8 CCRx and independent GPIO port/pin pairs. All accesses are single-write atomic operations on Cortex-M4.

## 5. Serial Command Protocol

### 5.1 Command Format (Text-based)

```
Format:   <CMD><ARGS>\r\n
Encoding: ASCII, case-insensitive
Response: "OK\r\n" or "ERR\r\n" or numeric value
```

### 5.2 Command Table

| Command | Description | Example | Response |
|---------|-------------|---------|----------|
| `M<id><+/-><duty>` | Set motor duty | `M1+50` | `OK\r\n` |
| `STOP` | Emergency stop all | `STOP` | `OK\r\n` |
| `ENC<id>` | Read encoder | `ENC1` | `12345\r\n` |
| `ENCALL` | Read all encoders | `ENCALL` | `E1:100 E2:200 E3:300 E4:400\r\n` |

- `<id>` = 1-4
- `duty` = 0-100
- Whitespace is stripped before parsing

### 5.3 Ring Buffer

```c
void    RingBuf_Init(void);
void    RingBuf_PutChar(uint8_t c);            // ISR-safe (no locking needed for single producer)
int     RingBuf_GetLine(char *buf, int timeout); // blocking read, returns 0=empty/timeout, 1=got line
```

- Size: 128 bytes
- ISR puts chars into buffer
- Task reads complete lines (delimited by `\n`)

## 6. FreeRTOS Task Plan

| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| defaultTask | Normal | 128*4 | Empty (user space) |
| led_test | Low | 128*4 | Blink PD5 (unchanged) |
| **SerialCmdTask** | Normal | 256*4 | Parse serial commands → Motor API |

SerialCmdTask created in `MX_FREERTOS_Init()` alongside existing tasks.

## 7. File Modifications

### 7.1 New Files

| File | Lines (est.) | Description |
|------|-------------|-------------|
| `motor.h` | ~50 | API declarations + Motor_ID_t enum + Motor_Context_t |
| `motor.c` | ~150 | Init all 4 motors, SetDuty, Stop, GetEncoder |
| `ringbuf.h` | ~15 | Ring buffer API |
| `ringbuf.c` | ~50 | Ring buffer with ISR-safe write |
| `serial_cmd.h` | ~10 | SerialCmdTask declaration |
| `serial_cmd.c` | ~80 | Command parser + FreeRTOS task function |

### 7.2 Modified Files

| File | Change |
|------|--------|
| `main.c` | Add `#include "motor.h"`, call `Motor_InitAll()` in USER CODE 2 |
| `main.c` | Start UART RX: call `HAL_UART_Receive_IT(&huart3, &rx_byte, 1)` in USER CODE 2 |
| `main.c` | Implement `HAL_UART_RxCpltCallback` to call `RingBuf_PutChar(rx_byte)` and re-arm RX |
| `freertos.c` | Add `#include "serial_cmd.h"`, create SerialCmdTask in MX_FREERTOS_Init |

## 8. Error Handling

| Scenario | Handling |
|----------|----------|
| Invalid motor ID (not 1-4) | `Motor_SetDuty` returns -1, caller ignores |
| duty out of range | Internally clamped to [-100, 100] |
| Ring buffer full | Drop oldest byte (prevent ISR blocking) |
| Unknown serial command | Reply "ERR\r\n" |
| Motor_DeInitAll | Stop all PWM + set all DIR pins LOW |

## 9. Future Expansion (NOT in this implementation)

- PID speed/position control loop
- Host-side binary protocol (for the planned PC debug tool)
- Motor_SetSpeed() with RPM target via encoder feedback
- Current sensing / stall detection

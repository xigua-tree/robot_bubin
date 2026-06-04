# 编码器 + 电机 + PID + 蓝牙 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在裸机模板上实现 4 路电机速度闭环控制（TB6612 驱动 + TIM 编码器 + 增量式 PID）和蓝牙协议串口通信。

**Architecture:** 分层模块化设计。底层：encoder（编码器读取）、motor（TB6612 PWM 控制）、ringbuf（中断安全环形缓冲）。中层：pid（增量式速度环算法）、bt_proto（A5/5A 帧协议解析/打包）。顶层：speed_ctrl（1kHz 控制循环，通过 TIM8 更新中断分频驱动）、main.c（协议处理 + TX 上报）。USART3 RXNE ISR 直接写 RingBuf，TIM8 ISR 以 10:1 分频触发控制标志位。

**Tech Stack:** STM32F407VET6, HAL 库, arm-none-eabi-gcc, PlatformIO, 无 RTOS

**硬件映射：**

| 电机 | 方向(IN1/IN2) | 编码器 TIM | PWM 通道 | 
|------|---------------|-----------|----------|
| 1 | PE14, PE15 | TIM1 (PA8,PA9) | TIM8_CH1 (PC6) |
| 2 | PE0, PE1 | TIM2 (PA0,PA1) | TIM8_CH3 (PC8) |
| 3 | PE12, PE13 | TIM3 (PA6,PA7) | TIM8_CH2 (PC7) |
| 4 | PB8, PB9 | TIM4 (PB6,PB7) | TIM8_CH4 (PC9) |

LED: PD5（收到有效包时闪烁）

**协议格式：**
- RX (BT→MCU, 19字节): `A5 | float speed | float Kp | float Ki | float Kd | checksum | 5A`
- TX (MCU→BT, 7字节): `A5 | float speed | checksum | 5A`
- checksum = 所有数据字节之和的低8位

**文件结构：**
- 新建: `Core/Inc/encoder.h`, `Core/Src/encoder.c` — 编码器抽象
- 新建: `Core/Inc/motor.h`, `Core/Src/motor.c` — TB6612 电机驱动
- 新建: `Core/Inc/pid.h`, `Core/Src/pid.c` — 增量式 PID
- 新建: `Core/Inc/ringbuf.h`, `Core/Src/ringbuf.c` — 环形缓冲区
- 新建: `Core/Inc/bt_proto.h`, `Core/Src/bt_proto.c` — 蓝牙协议
- 新建: `Core/Inc/speed_ctrl.h`, `Core/Src/speed_ctrl.c` — 速度环控制
- 修改: `Core/Src/main.c` — 集成调度
- 修改: `Core/Src/stm32f4xx_it.c` — TIM8 + USART3 ISR

---

### Task 1: ringbuf — 环形缓冲区

**Files:**
- Create: `Core/Inc/ringbuf.h`
- Create: `Core/Src/ringbuf.c`

- [ ] **Step 1: Write ringbuf.h**

```c
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

#define RINGBUF_SIZE 256

typedef struct {
    uint8_t buf[RINGBUF_SIZE];
    volatile uint16_t head;  /* ISR writes */
    uint16_t tail;           /* main loop reads */
} RingBuf_t;

void RingBuf_Init(RingBuf_t *rb);
bool RingBuf_Put(RingBuf_t *rb, uint8_t byte);     /* ISR-safe, single producer */
bool RingBuf_Get(RingBuf_t *rb, uint8_t *byte);     /* main loop consumer */
uint16_t RingBuf_Available(RingBuf_t *rb);
void RingBuf_Flush(RingBuf_t *rb);

#endif
```

- [ ] **Step 2: Write ringbuf.c**

```c
#include "ringbuf.h"

void RingBuf_Init(RingBuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuf_Put(RingBuf_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % RINGBUF_SIZE;
    if (next == rb->tail) {
        return false;  /* full */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

bool RingBuf_Get(RingBuf_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) {
        return false;  /* empty */
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % RINGBUF_SIZE;
    return true;
}

uint16_t RingBuf_Available(RingBuf_t *rb)
{
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return RINGBUF_SIZE - rb->tail + rb->head;
}

void RingBuf_Flush(RingBuf_t *rb)
{
    rb->tail = rb->head;
}
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 2: encoder — 编码器接口

**Files:**
- Create: `Core/Inc/encoder.h`
- Create: `Core/Src/encoder.c`

**依赖:** 外部 `TIM_HandleTypeDef htim1-4`（已在 tim.c 中定义并初始化）

- [ ] **Step 1: Write encoder.h**

```c
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "main.h"

/* 编码器线数（电机旋转一圈的脉冲数，4倍频前） */
#define ENCODER_PPR 11

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t  accum;       /* 32位累计值，处理16位溢出 */
    int16_t  last_raw;    /* 上一次原始值 */
    float    speed_rpm;   /* 当前速度 RPM */
} Encoder_t;

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder_t *enc);       /* 调用此函数计算速度（需周期性调用） */
int32_t Encoder_GetCount(Encoder_t *enc);
float Encoder_GetSpeed(Encoder_t *enc);

#endif
```

- [ ] **Step 2: Write encoder.c**

```c
#include "encoder.h"

/* 编码器 4 倍频：每个脉冲产生 4 个计数边沿 */
#define ENCODER_CPR (ENCODER_PPR * 4)

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->accum = 0;
    enc->last_raw = 0;
    enc->speed_rpm = 0.0f;

    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_t *enc)
{
    /* 读取当前16位计数值 */
    int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* 处理16位溢出（差值法，自动处理周期性回绕） */
    int16_t delta = raw - enc->last_raw;
    enc->accum += delta;
    enc->last_raw = raw;
}

int32_t Encoder_GetCount(Encoder_t *enc)
{
    return enc->accum;
}

float Encoder_GetSpeed(Encoder_t *enc)
{
    return enc->speed_rpm;
}
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 3: motor — TB6612 电机驱动

**Files:**
- Create: `Core/Inc/motor.h`
- Create: `Core/Src/motor.c`

**硬件映射：**
- 电机1: IN1=PE14, IN2=PE15, PWM=TIM8_CH1, htim8
- 电机2: IN1=PE0, IN2=PE1, PWM=TIM8_CH3, htim8
- 电机3: IN1=PE12, IN2=PE13, PWM=TIM8_CH2, htim8
- 电机4: IN1=PB8, IN2=PB9, PWM=TIM8_CH4, htim8

- [ ] **Step 1: Write motor.h**

```c
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "main.h"

/* TB6612 方向/刹车宏 */
#define MOTOR_CW   1
#define MOTOR_CCW -1
#define MOTOR_STOP 0

/* 最大 PWM 占空比（ARR = 16799） */
#define MOTOR_PWM_MAX 16799

typedef struct {
    /* 方向引脚 */
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    /* PWM（TIM8 各通道） */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} Motor_t;

extern Motor_t g_motors[4];

void Motor_InitAll(void);
/* duty: -1.0f ~ +1.0f，正=正转，负=反转 */
void Motor_SetDuty(Motor_t *m, float duty);
void Motor_Stop(Motor_t *m);
void Motor_Brake(Motor_t *m);

#endif
```

- [ ] **Step 2: Write motor.c**

```c
#include "motor.h"
#include "tim.h"

/* 电机1: PE14,PE15 + TIM8_CH1(PC6) */
/* 电机2: PE0,PE1   + TIM8_CH3(PC8) */
/* 电机3: PE12,PE13 + TIM8_CH2(PC7) */
/* 电机4: PB8,PB9   + TIM8_CH4(PC9) */
Motor_t g_motors[4];

void Motor_InitAll(void)
{
    /* 电机1 */
    g_motors[0].in1_port = GPIOE;
    g_motors[0].in1_pin  = GPIO_PIN_14;
    g_motors[0].in2_port = GPIOE;
    g_motors[0].in2_pin  = GPIO_PIN_15;
    g_motors[0].htim     = &htim8;
    g_motors[0].channel  = TIM_CHANNEL_1;

    /* 电机2 */
    g_motors[1].in1_port = GPIOE;
    g_motors[1].in1_pin  = GPIO_PIN_0;
    g_motors[1].in2_port = GPIOE;
    g_motors[1].in2_pin  = GPIO_PIN_1;
    g_motors[1].htim     = &htim8;
    g_motors[1].channel  = TIM_CHANNEL_3;

    /* 电机3 */
    g_motors[2].in1_port = GPIOE;
    g_motors[2].in1_pin  = GPIO_PIN_12;
    g_motors[2].in2_port = GPIOE;
    g_motors[2].in2_pin  = GPIO_PIN_13;
    g_motors[2].htim     = &htim8;
    g_motors[2].channel  = TIM_CHANNEL_2;

    /* 电机4 */
    g_motors[3].in1_port = GPIOB;
    g_motors[3].in1_pin  = GPIO_PIN_8;
    g_motors[3].in2_port = GPIOB;
    g_motors[3].in2_pin  = GPIO_PIN_9;
    g_motors[3].htim     = &htim8;
    g_motors[3].channel  = TIM_CHANNEL_4;

    /* 启动所有 PWM 通道 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* 初始停止 */
    for (int i = 0; i < 4; i++) {
        Motor_Stop(&g_motors[i]);
    }
}

void Motor_SetDuty(Motor_t *m, float duty)
{
    /* 限幅 */
    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint16_t pulse = (uint16_t)(fabsf(duty) * MOTOR_PWM_MAX);

    if (duty > 0.001f) {
        /* 正转: IN1=H, IN2=L */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    } else if (duty < -0.001f) {
        /* 反转: IN1=L, IN2=H */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    } else {
        /* 停止: IN1=L, IN2=L（TB6612 short brake） */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, pulse);
}

void Motor_Stop(Motor_t *m)
{
    Motor_SetDuty(m, 0.0f);
}

void Motor_Brake(Motor_t *m)
{
    /* TB6612: IN1=H, IN2=H = brake */
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
}

/* htim8 声明来自 tim.h */
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 4: pid — 增量式 PID

**Files:**
- Create: `Core/Inc/pid.h`
- Create: `Core/Src/pid.c`

**公式:** `Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))`

- [ ] **Step 1: Write pid.h**

```c
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float target;          /* 目标速度 */
    float error[3];        /* e(k), e(k-1), e(k-2) */
    float output;          /* 当前输出值 */
    float out_max;         /* 输出上限 */
    float out_min;         /* 输出下限 */
    float integral;        /* 积分累积（带限幅） */
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max);
float PID_Update(PID_t *pid, float measured);             /* 返回控制输出 */
void PID_SetTarget(PID_t *pid, float target);
void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd);
void PID_Reset(PID_t *pid);

#endif
```

- [ ] **Step 2: Write pid.c**

```c
#include "pid.h"
#include <string.h>

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max)
{
    memset(pid, 0, sizeof(PID_t));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_max = out_max;
    pid->out_min = -out_max;
    pid->integral_max = out_max * 0.3f;  /* 积分限幅为输出的30% */
}

float PID_Update(PID_t *pid, float measured)
{
    /* 计算误差 */
    float error = pid->target - measured;
    pid->error[2] = pid->error[1];   /* e(k-2) = e(k-1) */
    pid->error[1] = pid->error[0];   /* e(k-1) = e(k)   */
    pid->error[0] = error;           /* e(k)   = error   */

    /* 积分分离：误差较大时不累加积分，防止饱和 */
    if (fabsf(error) < pid->out_max * 0.5f) {
        pid->integral += error;
        if (pid->integral > pid->integral_max)  pid->integral = pid->integral_max;
        if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    } else {
        pid->integral = 0.0f;
    }

    /* 增量式 PID */
    /* Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2)) */
    float delta = pid->Kp * (pid->error[0] - pid->error[1])
                + pid->Ki * pid->error[0]
                + pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

    pid->output += delta;

    /* 输出限幅 */
    if (pid->output > pid->out_max)  pid->output = pid->out_max;
    if (pid->output < pid->out_min)  pid->output = pid->out_min;

    return pid->output;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 5: bt_proto — 蓝牙协议

**Files:**
- Create: `Core/Inc/bt_proto.h`
- Create: `Core/Src/bt_proto.c`

- [ ] **Step 1: Write bt_proto.h**

```c
#ifndef __BT_PROTO_H
#define __BT_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#define BT_FRAME_HEAD  0xA5
#define BT_FRAME_TAIL  0x5A

/* RX 包: A5 + speed(float) + Kp(float) + Ki(float) + Kd(float) + checksum + 5A */
#define BT_RX_DATA_LEN  16   /* 4个float */
#define BT_RX_PACKET_LEN 19  /* 1 + 16 + 1 + 1 */

/* TX 包: A5 + speed(float) + checksum + 5A */
#define BT_TX_DATA_LEN  4    /* 1个float */
#define BT_TX_PACKET_LEN 7   /* 1 + 4 + 1 + 1 */

/* 接收解析结果 */
typedef struct {
    float target_speed;
    float Kp;
    float Ki;
    float Kd;
} BT_RxPacket_t;

/* 发送数据 */
typedef struct {
    float speed;
} BT_TxPacket_t;

/* 帧解析状态机 */
typedef enum {
    BT_STATE_HEAD = 0,
    BT_STATE_DATA,
    BT_STATE_CHECKSUM,
    BT_STATE_TAIL
} BT_ParseState_t;

/* 解析一个字节，返回 true 表示收到完整有效帧 */
bool BT_Parse_Byte(uint8_t byte, BT_RxPacket_t *result);

/* 打包发送帧到 buffer，返回帧长度 */
uint8_t BT_Pack_Tx(const BT_TxPacket_t *pkt, uint8_t *buf);

#endif
```

- [ ] **Step 2: Write bt_proto.c**

```c
#include "bt_proto.h"
#include <string.h>

/* 状态机静态变量 */
static BT_ParseState_t s_state = BT_STATE_HEAD;
static uint8_t  s_data_buf[BT_RX_DATA_LEN];
static uint8_t  s_data_idx;
static uint8_t  s_checksum;

bool BT_Parse_Byte(uint8_t byte, BT_RxPacket_t *result)
{
    switch (s_state) {
    case BT_STATE_HEAD:
        if (byte == BT_FRAME_HEAD) {
            s_data_idx = 0;
            s_checksum = 0;
            s_state = BT_STATE_DATA;
        }
        break;

    case BT_STATE_DATA:
        s_data_buf[s_data_idx++] = byte;
        if (s_data_idx >= BT_RX_DATA_LEN) {
            s_state = BT_STATE_CHECKSUM;
        }
        break;

    case BT_STATE_CHECKSUM:
        /* 计算数据校验和 */
        for (uint8_t i = 0; i < BT_RX_DATA_LEN; i++) {
            s_checksum += s_data_buf[i];
        }
        if (byte == s_checksum) {
            s_state = BT_STATE_TAIL;
        } else {
            s_state = BT_STATE_HEAD;  /* 校验失败，丢弃 */
        }
        break;

    case BT_STATE_TAIL:
        if (byte == BT_FRAME_TAIL) {
            /* 完整帧接收成功，解析数据 */
            memcpy(&result->target_speed, &s_data_buf[0],  4);
            memcpy(&result->Kp,           &s_data_buf[4],  4);
            memcpy(&result->Ki,           &s_data_buf[8],  4);
            memcpy(&result->Kd,           &s_data_buf[12], 4);
            s_state = BT_STATE_HEAD;
            return true;
        }
        s_state = BT_STATE_HEAD;  /* 包尾不匹配，丢弃 */
        break;
    }
    return false;
}

uint8_t BT_Pack_Tx(const BT_TxPacket_t *pkt, uint8_t *buf)
{
    buf[0] = BT_FRAME_HEAD;

    /* 速度 (4 bytes, little-endian) */
    memcpy(&buf[1], &pkt->speed, 4);

    /* 校验和（数据部分4字节之和的低8位） */
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        checksum += buf[1 + i];
    }
    buf[5] = checksum;

    buf[6] = BT_FRAME_TAIL;
    return BT_TX_PACKET_LEN;
}
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 6: speed_ctrl — 速度环控制

**Files:**
- Create: `Core/Inc/speed_ctrl.h`
- Create: `Core/Src/speed_ctrl.c`

**依赖:** encoder, motor, pid（前三个模块已完成）

- [ ] **Step 1: Write speed_ctrl.h**

```c
#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"

#define SPEED_CTRL_MOTOR_COUNT 4

/* 控制参数 */
#define SPEED_CTRL_FREQ_HZ     1000.0f   /* 1kHz 控制频率 */
#define SPEED_CTRL_DT          0.001f    /* 1ms */

/* 默认 PID 参数 */
#define DEFAULT_KP  0.5f
#define DEFAULT_KI  0.1f
#define DEFAULT_KD  0.01f

/* 控制标志位（TIM8 ISR 置1，主循环清零） */
extern volatile uint8_t g_speed_ctrl_flag;

void SpeedCtrl_Init(void);
void SpeedCtrl_1kHz_Tick(void);          /* 执行一次控制计算 */
void SpeedCtrl_SetTarget(uint8_t id, float rpm);
void SpeedCtrl_SetPID(uint8_t id, float Kp, float Ki, float Kd);
float SpeedCtrl_GetSpeed(uint8_t id);    /* 获取实际速度 */

#endif
```

- [ ] **Step 2: Write speed_ctrl.c**

```c
#include "speed_ctrl.h"
#include "tim.h"

/* 编码器 + PID + 电机 实例 */
static Encoder_t s_encoders[SPEED_CTRL_MOTOR_COUNT];
static PID_t     s_pids[SPEED_CTRL_MOTOR_COUNT];

/* 控制标志位：TIM8 ISR 每10次中断置1 */
volatile uint8_t g_speed_ctrl_flag = 0;

/* TIM 句柄来自 tim.h */

void SpeedCtrl_Init(void)
{
    /* 初始化编码器 */
    Encoder_Init(&s_encoders[0], &htim1);
    Encoder_Init(&s_encoders[1], &htim2);
    Encoder_Init(&s_encoders[2], &htim3);
    Encoder_Init(&s_encoders[3], &htim4);

    /* 初始化 PID */
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        PID_Init(&s_pids[i], DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 1.0f);
        PID_SetTarget(&s_pids[i], 0.0f);
    }
}

void SpeedCtrl_1kHz_Tick(void)
{
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        /* 1. 更新编码器计数和速度 */
        Encoder_Update(&s_encoders[i]);

        /* 2. 计算速度 (RPM) */
        /* speed = delta_count / CPR / dt * 60 */
        /* 此处假设 Encoder_Update 被 1kHz 调用 */
        int32_t count = Encoder_GetCount(&s_encoders[i]);
        /* 速度计算由 Encoder_Update 内部累积，此处用差分法 */
        static int32_t last_count[SPEED_CTRL_MOTOR_COUNT];
        int32_t delta = count - last_count[i];
        last_count[i] = count;

        /* RPM = (delta / CPR) / dt * 60 */
        /* dt = 0.001s, CPR = ENCODER_PPR * 4 */
        float rpm = (float)delta / (float)(ENCODER_PPR * 4) / SPEED_CTRL_DT * 60.0f;

        /* 保存速度到 encoder 结构体 */
        s_encoders[i].speed_rpm = rpm;

        /* 3. PID 控制 */
        float output = PID_Update(&s_pids[i], rpm);

        /* 4. 更新 PWM */
        Motor_SetDuty(&g_motors[i], output);
    }
}

void SpeedCtrl_SetTarget(uint8_t id, float rpm)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        PID_SetTarget(&s_pids[id], rpm);
    }
}

void SpeedCtrl_SetPID(uint8_t id, float Kp, float Ki, float Kd)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        PID_SetTunings(&s_pids[id], Kp, Ki, Kd);
    }
}

float SpeedCtrl_GetSpeed(uint8_t id)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        return Encoder_GetSpeed(&s_encoders[id]);
    }
    return 0.0f;
}
```

- [ ] **Step 3: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 7: stm32f4xx_it.c — ISR 集成

**Files:**
- Modify: `Core/Src/stm32f4xx_it.c`

**变更：**
1. USART3_IRQHandler: 添加 RXNE 直接读取 → RingBuf_Put
2. TIM8_UP_TIM13_IRQHandler: 添加 10 分频计数器 → 置位 g_speed_ctrl_flag

- [ ] **Step 1: 修改 stm32f4xx_it.c**

在 `/* USER CODE BEGIN Includes */` 段：
```c
/* USER CODE BEGIN Includes */
#include "ringbuf.h"
#include "speed_ctrl.h"
/* USER CODE END Includes */
```

在 `/* USER CODE BEGIN PV */` 段：
```c
/* USER CODE BEGIN PV */
static RingBuf_t s_uart_rx_rb;
static volatile uint8_t s_tim8_divider;  /* TIM8 10分频计数器 */
/* USER CODE END PV */
```

在 `/* USER CODE BEGIN 0 */` 段：
```c
/* USER CODE BEGIN 0 */
RingBuf_t *Get_UART_RxRingBuf(void)
{
    return &s_uart_rx_rb;
}
/* USER CODE END 0 */
```

在 `USART3_IRQHandler` 的 `/* USER CODE BEGIN USART3_IRQn 0 */` 段：
```c
  /* USER CODE BEGIN USART3_IRQn 0 */
  /* 直接读取 RXNE 标志，绕过 HAL（保证低延迟） */
  if (USART3->SR & USART_SR_RXNE) {
      uint8_t c = (uint8_t)(USART3->DR & 0xFF);
      RingBuf_Put(&s_uart_rx_rb, c);
      return;  /* 已处理，不进入 HAL_UART_IRQHandler */
  }
  /* 清除溢出标志 */
  if (USART3->SR & USART_SR_ORE) {
      (void)USART3->DR;
      (void)USART3->SR;
  }
  /* USER CODE END USART3_IRQn 0 */
```

在 `TIM8_UP_TIM13_IRQHandler` 的 `/* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */` 段：
```c
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
  if (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
      /* 10kHz → 1kHz 分频 */
      s_tim8_divider++;
      if (s_tim8_divider >= 10) {
          s_tim8_divider = 0;
          g_speed_ctrl_flag = 1;
      }
      return;  /* 已处理，不进入 HAL_TIM_IRQHandler */
  }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
```

注意：`TIM8_UP_TIM13_IRQHandler` 中的 `HAL_TIM_IRQHandler(&htim8);` 需要保留在 USER CODE END 之后（不删除），以处理溢出中断之外的其他情况。

- [ ] **Step 2: 编译验证** — `pio run`，预期 SUCCESS

---

### Task 8: main.c — 主循环集成

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: 修改 main.c**

在 `/* USER CODE BEGIN Includes */` 段：
```c
/* USER CODE BEGIN Includes */
#include "ringbuf.h"
#include "bt_proto.h"
#include "speed_ctrl.h"
#include "motor.h"
/* USER CODE END Includes */
```

在 `/* USER CODE BEGIN PD */` 段：
```c
/* USER CODE BEGIN PD */
#define LED_PIN  GPIO_PIN_5
#define LED_PORT GPIOD
/* USER CODE END PD */
```

在 `/* USER CODE BEGIN PV */` 段：
```c
/* USER CODE BEGIN PV */
static BT_RxPacket_t s_rx_pkt;
static BT_TxPacket_t s_tx_pkt;
static uint8_t s_tx_buf[BT_TX_PACKET_LEN];
static uint8_t s_tx_len;
static uint32_t s_led_off_tick;  /* LED 闪烁计时 */
/* USER CODE END PV */
```

在 `/* USER CODE BEGIN PFP */` 段：
```c
/* USER CODE BEGIN PFP */
extern RingBuf_t *Get_UART_RxRingBuf(void);
/* USER CODE END PFP */
```

在 `/* USER CODE BEGIN 2 */` 段：
```c
  /* USER CODE BEGIN 2 */
  Motor_InitAll();
  SpeedCtrl_Init();

  /* 启用 TIM8 更新中断（PWM 已在 Motor_InitAll 中启动） */
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

  /* 使能 USART3 RXNE 中断 */
  USART3->CR1 |= USART_CR1_RXNEIE;

  /* LED 初始状态 */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  s_led_off_tick = 0;
  /* USER CODE END 2 */
```

在 `/* USER CODE BEGIN WHILE */` 段保持 `while (1) {` 不变，所有循环体代码放入 `/* USER CODE BEGIN 3 */` 段：

```c
    /* USER CODE BEGIN 3 */
    /* ---- LED 闪烁管理 ---- */
    if (s_led_off_tick > 0 && HAL_GetTick() >= s_led_off_tick) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        s_led_off_tick = 0;
    }

    /* ---- 协议解析 ---- */
    RingBuf_t *rb = Get_UART_RxRingBuf();
    uint8_t byte;
    if (RingBuf_Get(rb, &byte)) {
        if (BT_Parse_Byte(byte, &s_rx_pkt)) {
            /* 有效帧收到 → LED 亮 200ms */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            s_led_off_tick = HAL_GetTick() + 200;

            /* 应用 PID 参数和速度指令（对4个电机统一设置） */
            for (int i = 0; i < 4; i++) {
                SpeedCtrl_SetPID(i, s_rx_pkt.Kp, s_rx_pkt.Ki, s_rx_pkt.Kd);
                SpeedCtrl_SetTarget(i, s_rx_pkt.target_speed);
            }
        }
    }

    /* ---- 速度控制（由 TIM8 ISR 标志驱动） ---- */
    if (g_speed_ctrl_flag) {
        SpeedCtrl_1kHz_Tick();
        g_speed_ctrl_flag = 0;
    }

    /* ---- 周期上报速度（约 100ms 一次） ---- */
    {
        static uint32_t last_tx_tick = 0;
        if (HAL_GetTick() - last_tx_tick >= 100) {
            last_tx_tick = HAL_GetTick();
            /* 发送电机1的速度 */
            s_tx_pkt.speed = SpeedCtrl_GetSpeed(0);
            s_tx_len = BT_Pack_Tx(&s_tx_pkt, s_tx_buf);
            HAL_UART_Transmit(&huart3, s_tx_buf, s_tx_len, 100);
        }
    }
    /* USER CODE END 3 */
```

- [ ] **Step 2: 编译验证** — 完整编译

Run: `pio run`
Expected: SUCCESS, 0 errors, 0 warnings

---

### Task 9: 最终验证

- [ ] **Step 1: 完整编译**

Run: `"C:/Users/10369/.platformio/penv/Scripts/pio.exe" run`
Expected: SUCCESS

- [ ] **Step 2: 检查文件清单**

```bash
ls Core/Src/encoder.c Core/Src/motor.c Core/Src/pid.c Core/Src/ringbuf.c Core/Src/bt_proto.c Core/Src/speed_ctrl.c
ls Core/Inc/encoder.h Core/Inc/motor.h Core/Inc/pid.h Core/Inc/ringbuf.h Core/Inc/bt_proto.h Core/Inc/speed_ctrl.h
```

- [ ] **Step 3: Git commit**

```bash
git add Core/Src/encoder.c Core/Inc/encoder.h
git add Core/Src/motor.c Core/Inc/motor.h
git add Core/Src/pid.c Core/Inc/pid.h
git add Core/Src/ringbuf.c Core/Inc/ringbuf.h
git add Core/Src/bt_proto.c Core/Inc/bt_proto.h
git add Core/Src/speed_ctrl.c Core/Inc/speed_ctrl.h
git add Core/Src/main.c Core/Src/stm32f4xx_it.c
git commit -m "feat: encoder, motor(TB6612), PID speed control, bluetooth protocol"
```

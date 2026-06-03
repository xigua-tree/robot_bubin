# Serial Protocol Refactoring Design

**Date:** 2026-06-03
**Project:** test_for_f407 (STM32F407 + FreeRTOS + PlatformIO)
**Topic:** 串口协议从文本格式重构为二进制帧格式

---

## 1. Motivation

当前文本协议（`M1+50\r\n` / `OK\r\n`）问题：
- 无帧边界保护，数据中出现 `OK` 等字节会误解析
- 无校验，传输错误无法检测
- 命令扩展需修改解析器 if-else 链，维护性差
- 上位机解析文本比解析二进制帧复杂

## 2. Frame Format

### 2.1 General Structure

```
主机 → MCU (请求帧) 和 MCU → 主机 (应答帧) 共用同一格式:

┌──────┬──────┬──────┬──────┬──────────┬──────┐
│  CC  │  33  │ CMD  │ LEN  │   DATA   │  DD  │
│帧头1 │帧头2 │命令字│数据长│  N 字节  │帧尾  │
│ 1B   │ 1B   │ 1B   │ 1B   │ N 字节   │ 1B   │
└──────┴──────┴──────┴──────┴──────────┴──────┘
```

最小帧长 5 字节（LEN=0），最大帧长 `5 + MAX_DATA_LEN`。

| 字段 | 偏移 | 字节 | 说明 |
|------|------|------|------|
| HEAD1 | 0 | 1 | 固定 `0xCC` |
| HEAD2 | 1 | 1 | 固定 `0x33` |
| CMD | 2 | 1 | 命令字 |
| LEN | 3 | 1 | DATA 段字节数，范围 `0 ~ MAX_DATA_LEN` |
| DATA | 4 | LEN | 命令参数，可为空 |
| TAIL | 4+LEN | 1 | 固定 `0xDD` |

常量定义：
```c
#define PROTO_HEAD1       0xCC
#define PROTO_HEAD2       0x33
#define PROTO_TAIL        0xDD
#define MAX_DATA_LEN      32
#define FRAME_BUF_SIZE    (5 + MAX_DATA_LEN)   /* 37 bytes */
```

### 2.2 Frame Parsing Rules

- 用 LEN 字段决定帧尾位置，不在 DATA 中搜索 `0xDD`
- 如果 LEN > `MAX_DATA_LEN`，丢弃帧，回到搜寻帧头状态
- 帧头匹配失败（非 `CC 33`）时逐字节滑动，不做超时处理
- `0xCC` 或 `0x33` 出现在 DATA 段中不会干扰解析（由 LEN 保证边界）

## 3. Command Set

### 3.1 CMD 0x01 — Set Motor Duty

```
请求: CC 33 01 02 [motor_id] [duty] DD       (7 bytes)
应答: CC 33 01 00 DD                          (5 bytes, 成功确认)
错误: CC 33 FF 01 03 DD                        (motor_id 无效)
```

| 字段 | 字节 | 类型 | 说明 |
|------|------|------|------|
| motor_id | 1 | uint8_t | 0=M1, 1=M2, 2=M3, 3=M4 |
| duty | 1 | int8_t | -100 ~ +100（超出自动 clamp） |

### 3.2 CMD 0x02 — Read Single Encoder

```
请求: CC 33 02 01 [motor_id] DD               (6 bytes)
应答: CC 33 02 04 [enc0 enc1 enc2 enc3] DD     (9 bytes)
错误: CC 33 FF 01 03 DD                        (motor_id 无效)
```

| 字段 | 字节 | 类型 | 说明 |
|------|------|------|------|
| motor_id | 1 | uint8_t | 0~3 |
| enc | 4 | int32_t | 编码器计数值，小端序 (LSB first) |

### 3.3 CMD 0x03 — Emergency Stop

```
请求: CC 33 03 00 DD                          (5 bytes)
应答: CC 33 03 00 DD                          (5 bytes, 确认)
```

停止全部 4 路电机（coast stop: IN1=L, IN2=L, PWM=0）。

### 3.4 CMD 0x04 — Read All Encoders

```
请求: CC 33 04 00 DD                          (5 bytes)
应答: CC 33 04 10 [e1_4B] [e2_4B] [e3_4B] [e4_4B] DD   (21 bytes)
```

DATA 段 16 字节：4 个 int32_t 小端序，依次为电机 1~4 编码器值。

### 3.5 CMD 0xFF — Error Response (MCU → Host)

```
应答: CC 33 FF 01 [errcode] DD                 (6 bytes)
```

| errcode | 含义 |
|---------|------|
| `0x01` | 未知命令字 (CMD not recognized) |
| `0x02` | LEN 不匹配 (LEN != expected for this CMD) |
| `0x03` | 无效 motor_id (不在 0~3 范围) |

### 3.6 Command Quick Reference

| CMD | 功能 | 请求 LEN | 应答 LEN | 应答内容 |
|-----|------|---------|---------|---------|
| `0x01` | 设置占空比 | 2 | 0 | 空确认 |
| `0x02` | 读单个编码器 | 1 | 4 | int32_t LE |
| `0x03` | 急停 | 0 | 0 | 空确认 |
| `0x04` | 读全部编码器 | 0 | 16 | 4×int32_t LE |
| `0xFF` | 错误 | 1 | — | errcode（仅 MCU→主机） |

### 3.7 Reserved Command Space for Future Expansion

| CMD 范围 | 用途 |
|----------|------|
| `0x10 ~ 0x1F` | PID 参数读写 |
| `0x20 ~ 0x2F` | 速度环 / 位置环控制 |
| `0x30 ~ 0x3F` | Flash 配置存储 |
| `0x40 ~ 0x4F` | 系统状态查询（电压、温度等） |

## 4. Software Architecture

### 4.1 Layering

```
serial_cmd.c (FreeRTOS 任务 + 帧接收状态机)
    │ 调用 RingBuf_GetByte() 逐字节读
    │ 状态机解析完整帧 → 调用 proto_parse()
    │ dispatch: 根据 CMD 调用 motor API
    │ 构建应答帧 → UART 发送
    │
    ├── protocol.h / protocol.c (纯协议层)
    │   ├── proto_pack_xxx()      打包各类帧到 byte[]
    │   ├── proto_get_xxx()       从帧中提取字段值
    │   └── proto_validate()      校验帧头帧尾 + LEN
    │   无 HAL / FreeRTOS 依赖
    │
    └── motor.h (电机库，不变)
        ├── Motor_SetDuty()
        ├── Motor_GetEncoder()
        └── Motor_StopAll()
```

### 4.2 File Changes

| File | Status | Responsibility |
|------|--------|---------------|
| `Core/Inc/protocol.h` | NEW | 帧常量、命令字、打包/解析 API 声明 |
| `Core/Src/protocol.c` | NEW | 帧构造、字段提取、校验 |
| `Core/Inc/ringbuf.h` | MODIFY | 新增 `RingBuf_GetByte()` 声明 |
| `Core/Src/ringbuf.c` | MODIFY | 新增 `RingBuf_GetByte()` 实现 |
| `Core/Src/serial_cmd.c` | REWRITE | 只保留 FreeRTOS 任务 + 帧接收状态机 |

### 4.3 protocol.h API

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

/* Pack functions: encode fields into frame buffer, return total frame length */
int proto_pack_set_duty(uint8_t *buf, uint8_t motor_id, int8_t duty);
int proto_pack_duty_ack(uint8_t *buf);
int proto_pack_get_encoder_req(uint8_t *buf, uint8_t motor_id);
int proto_pack_get_encoder_resp(uint8_t *buf, int32_t enc_val);
int proto_pack_estop_req(uint8_t *buf);
int proto_pack_estop_ack(uint8_t *buf);
int proto_pack_get_enc_all_req(uint8_t *buf);
int proto_pack_get_enc_all_resp(uint8_t *buf, const int32_t enc_vals[4]);
int proto_pack_error(uint8_t *buf, uint8_t errcode);

/* Getters: extract fields from a validated frame at offset 2 (CMD) */
uint8_t proto_get_cmd(const uint8_t *frame);
uint8_t proto_get_len(const uint8_t *frame);
const uint8_t *proto_get_data(const uint8_t *frame);

/* Validation: check HEAD1/HEAD2 at [0]/[1] and TAIL at [4+LEN] */
int proto_validate_frame(const uint8_t *buf, int buf_len);

#endif /* __PROTOCOL_H__ */
```

### 4.4 Ring Buffer Extension

新增逐字节读取接口供帧状态机使用：

```c
/* ringbuf.h 新增 */
int RingBuf_GetByte(uint8_t *c, uint32_t timeout_ms);
```

```c
/* ringbuf.c 新增实现 */
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

保留 `RingBuf_GetLine()` 不动，向前兼容。

### 4.5 Frame Receive State Machine (serial_cmd.c)

```
            ┌─────────┐
            │ WAIT_H1 │ ← 等待 0xCC
            │ (state 0)│
            └────┬────┘
                 │ byte == 0xCC?
          ┌──────┴──────┐
          ▼ no           ▼ yes
    ┌─────────┐    ┌─────────┐
    │ WAIT_H1 │    │ WAIT_H2 │ ← 等待 0x33
    └─────────┘    └────┬────┘
                        │ byte?
              ┌─────────┼─────────┐
              ▼ 0xCC    ▼ 0x33    ▼ other
         stay H2   ┌─────────┐  ┌─────────┐
                   │ GET_CMD │  │ WAIT_H1 │ ← 重来
                   └────┬────┘  └─────────┘
                        │
                   ┌─────────┐
                   │ GET_LEN │
                   └────┬────┘
                        │ LEN ≤ MAX_DATA_LEN?
                   ┌────┴────┐
                   ▼ yes     ▼ no
             ┌──────────┐  ┌─────────┐
             │ GET_DATA │  │ WAIT_H1 │ ← 丢弃
             │ (LEN 字节)│  └─────────┘
             └────┬─────┘
                  │
             ┌──────────┐
             │ GET_TAIL │ ← 期望 0xDD
             └────┬─────┘
                  │
            ┌─────┴─────┐
            ▼ match      ▼ no match
      ┌───────────┐  ┌─────────┐
      │ DISPATCH  │  │ WAIT_H1 │ ← 丢弃
      │ (调用motor)│  └─────────┘
      │ (发送应答) │
      └─────┬─────┘
            │
            ▼
      ┌─────────┐
      │ WAIT_H1 │ ← 回到开始
      └─────────┘
```

状态机特点：
- `0xCC` 出现在非帧头位置时自动重同步（在 WAIT_H2 状态收到 0xCC 时保持 H2 状态而非跳回）
- LEN 超标时立即丢弃，防止缓冲区溢出
- 帧尾不匹配时丢弃整帧，不影响后续帧

### 4.6 Data Flow

```
USART3 RX ISR
  └─ HAL_UART_RxCpltCallback
       └─ RingBuf_PutChar(rx_byte)    ← ISR 安全写入

SerialCmdTask (FreeRTOS)
  └─ 状态机循环:
       RingBuf_GetByte(&c, 100)        ← 逐字节超时读取
       → 状态转移
       → DISPATCH: parse → motor API → pack response
       → HAL_UART_Transmit(response)   ← 发送应答
```

## 5. Error Handling

| Scenario | Behavior |
|----------|----------|
| 收到无效 CMD | 返回 `CC 33 FF 01 01 DD` |
| LEN 与命令期望不匹配 | 返回 `CC 33 FF 01 02 DD` |
| motor_id 不在 0~3 | 返回 `CC 33 FF 01 03 DD` |
| LEN > MAX_DATA_LEN | 丢弃帧，不发错误（恶意帧不确认） |
| 帧尾不匹配 | 丢弃帧，不发错误（等待下一帧） |
| `0xCC`/`0x33`/`0xDD` 出现在 DATA 中 | 正常，LEN 保证不被误解析 |
| 帧中途超时（字节间超时 100ms） | 状态机复位到 WAIT_H1 |

## 6. Examples

### Example 1: Set Motor 1 duty to +50%
```
Host → MCU: CC 33 01 02 00 32 DD
MCU → Host: CC 33 01 00 DD
```

### Example 2: Set Motor 2 duty to -30%
```
Host → MCU: CC 33 01 02 01 E2 DD
MCU → Host: CC 33 01 00 DD
```

### Example 3: Read Motor 3 encoder (value = 250000 = 0x0003D090)
```
Host → MCU: CC 33 02 01 02 DD
MCU → Host: CC 33 02 04 90 D0 03 00 DD
```

### Example 4: Emergency stop
```
Host → MCU: CC 33 03 00 DD
MCU → Host: CC 33 03 00 DD
```

### Example 5: Read all encoders
```
Host → MCU: CC 33 04 00 DD
MCU → Host: CC 33 04 10 [e1_4B][e2_4B][e3_4B][e4_4B] DD
```

### Example 6: Invalid motor ID
```
Host → MCU: CC 33 01 02 05 32 DD     (motor_id=5, invalid)
MCU → Host: CC 33 FF 01 03 DD        (errcode=0x03)
```

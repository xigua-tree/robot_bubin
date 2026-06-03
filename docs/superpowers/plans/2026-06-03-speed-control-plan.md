# Speed Control Loop Implementation Plan

**Goal:** Add incremental PID-based speed control (velocity loop) with 1kHz FreeRTOS task.

---

### Task 1: pid.h + pid.c — Incremental PID Controller

Pure math library, no HAL/FreeRTOS deps.

### Task 2: speed_ctrl.h/c — Speed Control on 1kHz FreeRTOS Task

4x SpeedCtrl_t instances, encoder delta → RPM → PID → Motor_SetDuty.

### Task 3: motor.h/c — No changes (Motor_SetDuty already public)

### Task 4: protocol.h/c — Add CMD 0x10~0x14

| CMD | Function | Request | Response |
|-----|----------|---------|----------|
| 0x10 | Switch mode | `<id> <mode>` | ack |
| 0x11 | Set target RPM | `<id> <rpm_i16>` | ack |
| 0x12 | Set PID params | `<id> <kp> <ki> <kd>` (3×i16) | ack |
| 0x13 | Read RPM | `<id>` | `<rpm_i16>` |
| 0x14 | Read all RPM | — | 4×i16 LE |

### Task 5: serial_cmd.c — Dispatch new CMDs

### Task 6: freertos.c — Create SpeedCtrlTask (AboveNormal, 256×4 stack)

### Task 7: Build

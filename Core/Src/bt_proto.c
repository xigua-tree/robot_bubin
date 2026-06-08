#include "bt_proto.h"
#include <string.h>
#include "main.h"

/* IMU 欧拉角（由 main.c 中的 imu_update_task 更新） */
extern float g_roll, g_pitch, g_yaw;


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
            memcpy(&result->Vx, &s_data_buf[0],  4);
            memcpy(&result->Vy,           &s_data_buf[4],  4);
            memcpy(&result->target_speed,           &s_data_buf[8],  4);
            memcpy(&result->Kp,           &s_data_buf[12], 4);
            memcpy(&result->Ki,           &s_data_buf[16], 4);
            memcpy(&result->Kd,           &s_data_buf[20], 4);
            memcpy(&result->yaw,           &s_data_buf[24], 4);
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

    memcpy(&buf[1],  &pkt->count,      sizeof(pkt->count));
    memcpy(&buf[5],  &pkt->Encoder_l,  sizeof(pkt->Encoder_l));
    memcpy(&buf[9],  &pkt->Encoder_r,  sizeof(pkt->Encoder_r));
    memcpy(&buf[13], &pkt->Vx,      sizeof(pkt->Vx));
    memcpy(&buf[17], &pkt->Vy,      sizeof(pkt->Vy));
    memcpy(&buf[21], &pkt->yaw,       sizeof(pkt->yaw));
    memcpy(&buf[25], &pkt->wheel_rpm[0],      sizeof(pkt->wheel_rpm[0]));
    memcpy(&buf[29], &pkt->wheel_rpm[1],      sizeof(pkt->wheel_rpm[1]));
    memcpy(&buf[33], &pkt->wheel_rpm[2],      sizeof(pkt->wheel_rpm[2]));
    memcpy(&buf[37], &pkt->wheel_rpm[3],      sizeof(pkt->wheel_rpm[3]));
    memcpy(&buf[41], &pkt->speed,      sizeof(pkt->speed));

    /* 校验和（数据字节之和的低8位） */
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < BT_TX_DATA_LEN; i++) {
        checksum += buf[1 + i];
    }
    buf[BT_TX_DATA_LEN + 1] = checksum;

    buf[BT_TX_DATA_LEN + 2] = BT_FRAME_TAIL;
    return BT_TX_PACKET_LEN;
}

void blue_setparam_task()
{
        /* ---- 协议解析 ---- */
    RingBuf_t *rb = Get_UART_RxRingBuf();
    uint8_t byte;
    while (RingBuf_Get(rb, &byte)) {
        if (BT_Parse_Byte(byte, &s_rx_pkt)) {
            /* 应用 PID 参数和速度指令（对4个电机统一设置） */
                // for(int i = 0; i < 4; i++){
                //     // SpeedCtrl_SetPID(i,s_rx_pkt.Kp,s_rx_pkt.Ki,s_rx_pkt.Kd);
                //     SpeedCtrl_SetTarget(i, s_rx_pkt.target_speed);
                // }
                g_target_yaw_angle = s_rx_pkt.yaw;
                Vx = s_rx_pkt.Vx;
                Vy = s_rx_pkt.Vy;
                YawCtrl_SetPID(s_rx_pkt.Kp,s_rx_pkt.Ki,s_rx_pkt.Kd);
        }
    }
        /* ---- 周期上报速度（约 100ms 一次） ---- */
    {
        static uint32_t last_tx_tick = 0;
        if (HAL_GetTick() - last_tx_tick >= 100) {
            last_tx_tick = HAL_GetTick();
            /* 发送电机1的速度 */
            s_tx_pkt.count = tim8_counter;
            // s_tx_pkt.speed = g_encoders[0].speed_rpm;//wheel_rpm
            // s_tx_pkt.error = SpeedCtrl_GetError(0);
            s_tx_pkt.Encoder_l = encoderl_value;
            s_tx_pkt.Encoder_r = encoderr_value;
            s_tx_pkt.Vx = Vx;
            s_tx_pkt.Vy = Vy;
            s_tx_pkt.yaw   = g_yaw;
            // s_tx_pkt.wheel_rpm[0] = g_encoders[0].speed_rpm;
            // s_tx_pkt.wheel_rpm[1] = g_encoders[1].speed_rpm;
            s_tx_pkt.wheel_rpm[2] = g_encoders[2].speed_rpm;
            s_tx_pkt.wheel_rpm[3] = g_encoders[3].speed_rpm;
            s_tx_pkt.wheel_rpm[0] = omega;
            s_tx_pkt.wheel_rpm[1] = wheel_rpm[1];
            // s_tx_pkt.wheel_rpm[2] = wheel_rpm[2];
            // s_tx_pkt.wheel_rpm[3] = wheel_rpm[3];
            s_tx_pkt.speed = yaw_err;
            s_tx_len = BT_Pack_Tx(&s_tx_pkt, s_tx_buf);
            HAL_UART_Transmit(&huart3, s_tx_buf, s_tx_len, 100);
        }
    }
}